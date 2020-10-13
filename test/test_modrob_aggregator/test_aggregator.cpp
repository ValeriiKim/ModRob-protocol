#include "unity.h"
#include <modrob.hpp>
#include <string>
#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <cstdarg>
#include <array>
#include <string.h>
#include <bitset>
#include "lwrb.h"
#include "../lib/ArduinoJson.h"
#include "../uthash.h"
#include "../khash.h"
#include "modrob_aggregator.hpp"

#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1


using namespace modrob;

random_device rd;
mt19937 gen(rd());


uint8_t rand8Bit(){
    static uniform_int_distribution<> rand8(0, 255);
    return rand8(gen);
}

uint16_t rand16Bit(){
    static uniform_int_distribution<> rand16(0, 65535);
    return rand16(gen);
}

uint32_t rand32Bit(){
    static uniform_int_distribution<long long unsigned> rand32(0,0xFFFFFFFFFFFFFFFF);
    return rand32(gen);
}

float randFloat(){
    static uniform_real_distribution<float> randFloat(-1000.0F, 1000.0F);
    return randFloat(gen);
}

float randFloat_minmax(float min, float max){
    static uniform_real_distribution<float> randFloat(min, max);
    return randFloat(gen);
}

struct VariableInfo_2
{
    uint16_t moduleID;
    uint8_t  variableID{0};
    uint8_t  typeID;
    uint16_t hertz{0};
    float value;
    // uint16_t subscribeFromModuleID{0};
    // uint8_t  subscribeFromVariableID{0};
    bool isChanged = false;
    bool isLogged = false;
};



static uint64_t numberOfTestIteration = 1000000L;

/** Массив из очередей, имитирующий CAN шину. Тип элементов очереди - Modrob сообщение. 
 * Каждая очередь соответствует одному узлу, всего таких узлов - 65536, поскольку module ID 
 * имеет размер в 16 бит, что соответствует такому числу узлов. 
 * Примечание: очередь фактически не ограничена по размеру, поэтому если её нужно ограничивать, 
 * делать это надо вручную. 
*/
array<queue<ModrobMessage>, 65536> canBus = {};
uint16_t num_of_heartbeats{0};

/**
* Класс, эмулирующий работу CAN узла (для тестов)
*/
struct VirtualCanTransport: ModrobTransport {

    uint16_t modID;

    VirtualCanTransport(uint16_t _modID): modID(_modID){

    }

/* Отправка Modrob сообщения на виртуальную шину */
    void send(const ModrobMessage &msg) override {
        sendToBus(msg);
    }

/* Получение Modrob сообщения, принятого из виртуальной шины */
    bool receive(ModrobMessage &result) override {
        // Если буфер узла с modID пуст, то сообщений нет
        if(canBus[modID].empty()){
            return false;
        }
        ModrobMessage msg = canBus[modID].front(); // получаем первый элемент очереди
        canBus[modID].pop();                       // выталкиваем первый элемент очереди
        result = msg; 
        return true;
    }

/** Отправка сообщения на виртуальную шину. Поскольку в CAN сообщения приходят ко всем узлам, 
 * то мы должны пройтись по всем элементам массива (узлам шины), чтобы добавить в конец каждой 
 * очереди по новому сообщению 
 */
    static void sendToBus(const ModrobMessage &msg){
        if (msg.operationType == OperationType::HeartBeat) {
            num_of_heartbeats++;
            // cout << "heartbeat of modID: " << msg.moduleID
            //      << " with typeID: " << static_cast<int>(msg.typeID) << endl; 
        }
        for(auto & queue : canBus){
            queue.push(msg);
        }
    }

    static void clear(){
        for(auto& queue : canBus){
            while(!queue.empty()){
                queue.pop();
            }
        }
    }
};


void compose_hashID(int& id, uint16_t subModuleID, uint8_t subVariableID)
{
    id = ((subModuleID & 0xFFFFU) << 8U) | (subVariableID & 0xFF);
}


KHASH_MAP_INIT_INT(pub2, VariableInfo_2);
khash_t(pub2) *publishers = kh_init(pub2);

void khash_add(uint16_t pub_modID, uint8_t pub_varID, uint16_t pub_hertz)
{
    khint_t key;
    int id = 0;
    int ret;
    compose_hashID(id, pub_modID, pub_varID);
    key = kh_put(pub2, publishers, id, &ret);
    if (ret == 0) {
        cout << "The key is already present in table" << endl;
        return;
    }
    kh_value(publishers, key).moduleID = pub_modID;
    kh_value(publishers, key).variableID = pub_varID;
    kh_value(publishers, key).hertz = pub_hertz;
}

void khash_print()
{
    khint_t key;
    for (key = kh_begin(publishers); key != kh_end(publishers); ++key) {
        if (kh_exist(publishers, key)) {
            cout << "modID: " << kh_value(publishers, key).moduleID
                 << ", varID: " << static_cast<int>(kh_value(publishers, key).variableID) 
                 << ", frequency: " << kh_value(publishers, key).hertz << endl;
        }
    }
}

VariableInfo_2& khash_find(uint16_t modID_find, uint8_t varID_find, khint_t& check_key)
{
    int id_to_find;
    khint_t key;
    compose_hashID(id_to_find, modID_find, varID_find);
    key = kh_get(pub2, publishers, id_to_find);
    if (key == kh_end(publishers)) {
        cout << "No such key!" << endl;
    }
    check_key = key;
    cout << "FOUND modID: " << kh_value(publishers, key).moduleID
         << ", varID: " << static_cast<int>(kh_value(publishers, key).variableID) 
         << ", frequency: " << kh_value(publishers, key).hertz << endl;
    return kh_value(publishers, key);
}

void khash_delete(uint16_t modID_to_del, uint8_t varID_to_del)
{
    int id_to_del;
    khint_t key;
    compose_hashID(id_to_del, modID_to_del, varID_to_del);
    key = kh_get(pub2, publishers, id_to_del);
    kh_del(pub2, publishers, key);
}

void test_modrob_hash_tables()
{
    unsigned int key;
    khash_add(0, 1, 100);
    khash_add(1, 2, 120);
    khash_add(1, 5, 130);
    khash_add(2, 11, 140);
    khash_add(3, 1, 150);
    khash_print();
    TEST_ASSERT_EQUAL(5, kh_size(publishers));
    VariableInfo_2 var_find = khash_find(2, 11, key);
    TEST_ASSERT_EQUAL(2, var_find.moduleID);
    TEST_ASSERT_EQUAL(11, var_find.variableID);
    TEST_ASSERT_EQUAL(140, var_find.hertz);

    khash_delete(3, 1);
    TEST_ASSERT_EQUAL(4, kh_size(publishers));
    khash_print();

    cout << "Change frequency of variable 5 modID 1" << endl;
    VariableInfo_2& var_to_change = khash_find(1, 5, key);
    var_to_change.hertz = 999;
    TEST_ASSERT_EQUAL(999, khash_find(1, 5, key).hertz);
    khash_print();
}

float round2(float value)
    {
        float val = static_cast<int>(value * 100 + 0.5);
        return val / 100;
    }

struct VirtualSerialTransport: public JSONTransport
{
    char serial_buffer[256];

    VirtualSerialTransport()
    {}

    void send(const ModrobInfo& variable, AggregatorMessage type) override
    {
        StaticJsonDocument<256> doc;
        switch (type)
        {
        case AggregatorMessage::Logging:
            doc["modID"] = variable.moduleID;
            doc["varID"] = variable.variableID;
            doc["value"] = round2(variable.value);
            doc["hertz"] = variable.hertz;
            serializeJson(doc, serial_buffer);
            serial_buffer[strlen(serial_buffer)] = '\n';
            cout << serial_buffer;
            break;

        case AggregatorMessage::VarTable:
            doc["Table"] = "Publishers";
            doc["modID"] = variable.moduleID;
            doc["varID"] = variable.variableID;
            doc["value"] = round2(variable.value);
            doc["hertz"] = variable.hertz;
            serializeJson(doc, serial_buffer);
            serial_buffer[strlen(serial_buffer)] = '\n';
            cout << serial_buffer;
            break;

        case AggregatorMessage::NodeTable:
            doc["Table"] = "Nodes";
            doc["modID"] = variable.moduleID;
            doc["typeID"] = variable.typeID;
            switch (variable.status)
            {
            case NodeStatus::Off:
                doc["status"] = "Off";
                break;
            
            case NodeStatus::Passive:
                doc["status"] = "Passive";
                break;

            case NodeStatus::Active:
                doc["status"] = "Active";
                break;
            }
            doc["pubVarNum"] = variable.pubVarNum;
            serializeJson(doc, serial_buffer);
            serial_buffer[strlen(serial_buffer)] = '\n';
            cout << serial_buffer;
            break;
        }

        memset(serial_buffer, 0, sizeof(serial_buffer));
    }
    bool receive(ModrobInfo& result) override
    {};
};



void test_modrob_aggregator()
{
    unsigned int check_key;
    VirtualCanTransport::clear();
    VirtualCanTransport can0 = VirtualCanTransport(0);
    VirtualCanTransport can1 = VirtualCanTransport(1);
    VirtualCanTransport can2 = VirtualCanTransport(2);

    Aggregator broker(0, 0);
    VirtualSerialTransport virtual_transport;
    broker.setUpperTransport(virtual_transport);
    broker.setTransport(can0);
    broker.add_new_publisher(1, 1, 100);
    broker.add_new_publisher(2, 2, 200);
    broker.set_logging(1, 1);
    broker.set_logging(2, 2);

    Node node1 = Node(1, 1);
    node1.setTransport(can1);
    auto& var1 = node1.createVariable(1, "var1", "scalar", 0.0);

    Node node2 = Node(2, 2);
    node2.setTransport(can2);
    auto& var2 = node2.createVariable(2, "var2", "scalar", 0.0);

    auto pubNode1 = ModrobMessage::commandSetHertz(1, 1, 100);
    auto pubNode2 = ModrobMessage::commandSetHertz(2, 2, 200);
    auto setVar1  = ModrobMessage::commandSetValue(1, 1, 12.12);
    auto setVar2  = ModrobMessage::commandSetValue(2, 2, 32.4);

    VirtualCanTransport::sendToBus(pubNode1);
    VirtualCanTransport::sendToBus(pubNode2);
    // node1.run(1);
    // node2.run(1);
    // broker.run(1);

    // TEST_ASSERT_EQUAL(var1.hertz, 100);
    // TEST_ASSERT_EQUAL(var2.hertz, 0);
    // ModrobInfo& var_info1 = broker.find_publisher(1, 1, check_key);
    // TEST_ASSERT_EQUAL(var_info1.moduleID, 1);
    // TEST_ASSERT_EQUAL(var_info1.value, 0);
    // ModrobInfo& var_info2 = broker.find_publisher(2, 2, check_key);
    // TEST_ASSERT_EQUAL(var_info2.moduleID, 2);
    // TEST_ASSERT_EQUAL(var_info2.value, 0);
    

    // node1.run(2);
    // node2.run(2);
    // broker.run(2);

    // TEST_ASSERT_EQUAL(var1.hertz, 100);
    // TEST_ASSERT_EQUAL(var2.hertz, 200);
    // var_info1 = broker.find_publisher(1, 1, check_key);
    // TEST_ASSERT_EQUAL(var_info1.moduleID, 1);
    // TEST_ASSERT_EQUAL(var_info1.value, 0);
    // var_info2 = broker.find_publisher(2, 2, check_key);
    // TEST_ASSERT_EQUAL(var_info2.moduleID, 2);
    // TEST_ASSERT_EQUAL(var_info2.value, 0);

    VirtualCanTransport::sendToBus(setVar1);
    VirtualCanTransport::sendToBus(setVar2);

    for (long t = 0; t < 21000; t++)
    {
        node1.run(t);
        node2.run(t);
        broker.run(t);
    }
    broker.send_publishers_table();

}


void test_modrob_heartbeat()
{
    VirtualCanTransport::clear();
    VirtualCanTransport can0 = VirtualCanTransport(0);
    VirtualCanTransport can1 = VirtualCanTransport(1);
    VirtualCanTransport can2 = VirtualCanTransport(2);
    VirtualCanTransport can3 = VirtualCanTransport(3);
    VirtualSerialTransport virtual_transport;
    Aggregator broker(0, 0);
    Node node1 = Node(1, 1);
    Node node2 = Node(2, 2);
    Node node3 = Node(3, 3);
    broker.setUpperTransport(virtual_transport);
    broker.setTransport(can0);
    node1.setTransport(can1);
    node2.setTransport(can2);
    node3.setTransport(can3);
    
    for (long t = 0; t < 5*SECOND + 100; t++)
    {
        node1.run(t);
        node2.run(t);
        if (t == 2*SECOND) {
            broker.send_node_table();
            TEST_ASSERT_EQUAL(broker.get_nodes_num(), 2);
        }
        if (t > 3*SECOND) {
            node3.run(t);
        }
        broker.run(t);
    }
    // TEST_ASSERT_EQUAL(num_of_heartbeats, 15);
    cout << endl;
    broker.send_node_table();
    TEST_ASSERT_EQUAL(broker.get_nodes_num(), 3);
}

int main()
{
    UNITY_BEGIN();
    // RUN_TEST(test_modrob_hash_tables);
    RUN_TEST(test_modrob_aggregator);
    RUN_TEST(test_modrob_heartbeat);
    UNITY_END();

}