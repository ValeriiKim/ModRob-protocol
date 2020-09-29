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

struct CAN_Message
{
	unsigned int id;	   // 29 bit identifier
	unsigned char data[8]; // data field
	unsigned char len;	   // length of data field
	unsigned char format;  // format: standard or extended
	unsigned char type;	   // type: data or remote frame

/* Конструктор для создания пустого CAN сообщения */
	CAN_Message()
	{
		unsigned int id   = 0U;
		memset(data, 0, 8);
		unsigned char len = 8U;
		unsigned char format = STANDARD_FORMAT;
		unsigned char type = DATA_FRAME;
	}

/** Конструктор для создания CAN сообщения с определённым 
 * содержанием по умолчанию используем расширенный формат кадра
 * 
 *  @param _id      Message ID
 *  @param _data    Mesaage Data
 *  @param _len     Message Data length
 *  @param _type    Type of Data: EXTENDED_FORMAT or STANDARD_FORMAT
 *  @param _format  Data Format: DATA_FRAME or REMOTE_FRAME
 */
    CAN_Message(unsigned int _id, const unsigned char *_data, unsigned char _len = 8,
	            unsigned char _format = EXTENDED_FORMAT, unsigned char _type = DATA_FRAME)
	{
		id = _id;
		memcpy(data, _data, _len);
		len = _len & 0xF;
		format = _format;
		type = _type;
	}
};

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


void assert_messages_equal(const ModrobMessage& actual, const ModrobMessage& expected)
{
    TEST_ASSERT_EQUAL(actual.moduleID,         expected.moduleID);
    TEST_ASSERT_EQUAL(actual.variableID,       expected.variableID);
    TEST_ASSERT_EQUAL(actual.messageType,      expected.messageType);
    TEST_ASSERT_EQUAL(actual.value,            expected.value);
    TEST_ASSERT_EQUAL(actual.typeID,           expected.typeID);
    TEST_ASSERT_EQUAL(actual.targetVariableID, expected.targetVariableID);
    TEST_ASSERT_EQUAL(actual.operationType,    expected.operationType);
}


void test_size_of_structures(){
    cout << "Node: " << sizeof(Node) << " bytes" << endl;
    cout << "Variable: " << sizeof(Variable) << " bytes" << endl;
    cout << "VariableInfo: " << sizeof(VariableInfo) << " bytes" << endl;
    cout << "VariableInfo_2: " << sizeof(VariableInfo_2) << " bytes" << endl;
    cout << "ModrobMessage: " << sizeof(ModrobMessage) << " bytes" << endl;
}

void test_modrob_adding_variables(){
    Node node(1, 11);
    node.createVariable(0, "name1", "unit1", 0.0);
    node.createVariable(1, "name1", "unit1", 0.0);
    node.createVariable(2, "name1", "unit1", 0.0);
    node.createVariable(3, "name1", "unit1", 0.0);
    node.createVariable(4, "name1", "unit1", 0.0);
    TEST_ASSERT_EQUAL(node.variables.size(), 5);
}

/** Проверка соответствия между установкой и получением значения float 
 * modrob переменной 
*/
void test_modrob_get_set_var()
{
    Node node(10, 11);
    auto v = node.createVariable(0, "name1", "unit1", 5.0F);
    TEST_ASSERT(v.get() == 5.0F);
    v.set(10.0F);
    TEST_ASSERT(v.get() == 10.0F);
}

static uint64_t numberOfTestIteration = 1000000L;


/** Сравниваются поля исходного Modrob сообщения 
 * с полями сообщения, которое преобразуется в CAN фрейм и обратно 
*/
void test_modrob_messages_from_to()
{
    ModrobMessage m1 = {};
    ModrobMessage m2 = {};

    for(auto i = 0; i < numberOfTestIteration; i++)
    {
        m1.moduleID         = rand16Bit();
        m1.variableID       = rand8Bit();
        m1.messageType      = MessageType::Command;
        m1.value            = rand32Bit();
        m1.typeID           = rand8Bit();
        m1.targetVariableID = rand8Bit();
        m1.operationType    = OperationType ::SetSubscriptionAddress;

        unsigned int canID = 0;
        unsigned char data[8];

        m1.toCan(canID, data);
        m2.fromCan(canID, data);

        TEST_ASSERT_EQUAL(m1.moduleID,    m2.moduleID);
        TEST_ASSERT_EQUAL(m1.variableID,  m2.variableID);
        TEST_ASSERT_EQUAL(m1.messageType, m2.messageType);

        TEST_ASSERT_EQUAL(m1.value,            m2.value);
        TEST_ASSERT_EQUAL(m1.typeID,           m2.typeID);
        TEST_ASSERT_EQUAL(m1.targetVariableID, m2.targetVariableID);
        TEST_ASSERT_EQUAL(m1.operationType,    m2.operationType);
    }
}

void test_modrob_messages_from_to_value()
{
    ModrobMessage m1 = {};
    ModrobMessage m2 = {};
    for(auto i = 0; i < numberOfTestIteration; i++)
    {
        float new_value = randFloat();
        m1.setValue(new_value);
        unsigned int canID = 0;
        unsigned char data[8];

        m1.toCan(canID, data);
        m2.fromCan(canID, data);

        TEST_ASSERT(m2.getValue() == new_value);
    }
    // float value = 2.1;
    // m1.setValue(value);
    // unsigned int canID = 0;
    // unsigned char data[8];
    // m1.toCan(canID, data);
    // cout << std::bitset<8>(data[0]) << endl;
    // cout << std::bitset<8>(data[1]) << endl;
    // cout << std::bitset<8>(data[2]) << endl;
    // cout << std::bitset<8>(data[3]) << endl;
    // cout << std::bitset<8>(data[4]) << endl;
    // cout << std::bitset<8>(data[5]) << endl;
    // cout << std::bitset<8>(data[6]) << endl;

}


void test_modrob_messages_set_get_address()
{
    ModrobMessage m1 = {};
    uint16_t post_modID;
    uint8_t  post_varID;
    for(auto i = 0; i < numberOfTestIteration; i++)
    {
        uint16_t moduleID    = rand16Bit();
        uint8_t  variableID  = rand8Bit();
        m1.setTargetAddress(moduleID, variableID);
        m1.getTargetAddress(post_modID, post_varID);
        TEST_ASSERT_EQUAL(moduleID,   post_modID);
        TEST_ASSERT_EQUAL(variableID, post_varID);
    }
}

void test_modrob_messages_change_var()
{
    for(auto i = 0; i < numberOfTestIteration; i++)
    {
        uint16_t targetModID = rand16Bit();
        uint8_t  targetVarID = rand8Bit();
        float    targetValue = randFloat_minmax(-100000.0F, 100000.0F);
        ModrobMessage m1 = ModrobMessage::commandSetValue(targetModID, targetVarID, targetValue);
        TEST_ASSERT_EQUAL(m1.getValue(), targetValue);

        Node node = Node(targetModID, 0);
        Variable& var = node.createVariable(targetVarID, "var", "scalar", 0.0F);

        TEST_ASSERT(var.get() == 0.0F);
        node.setReceivedMessage(m1);
        node.run(0);

        TEST_ASSERT(var.isChanged);
        TEST_ASSERT_EQUAL(var.get(), targetValue);
        TEST_ASSERT(!var.isChanged);
    }   
}

void test_modrob_messages_change_hertz()
{
    for(auto i=0; i < numberOfTestIteration; i++){
        uint16_t targetModID = rand16Bit();
        uint8_t  targetVarID = rand8Bit();
        uint8_t  targetHertz = rand8Bit();

        ModrobMessage m1 = ModrobMessage::commandSetHertz(targetModID, targetVarID, targetHertz);

        Node node = Node(targetModID, 0);
        Variable& var = node.createVariable(targetVarID, "var", "scalar", 100.0F);

        TEST_ASSERT(var.hertz == 0);
        node.setReceivedMessage(m1);
        node.run(0);

        TEST_ASSERT(var.hertz == targetHertz);
    }
}

void test_modrob_messages_change_subscription_address()
{
    for(auto i = 0; i < numberOfTestIteration; i++)
    {
        uint16_t targetModID = rand16Bit();
        uint8_t  targetVarID = rand8Bit();

        uint16_t subModID = rand16Bit();
        uint8_t  subVarID = rand8Bit();

        ModrobMessage m1 = ModrobMessage::commandSetSubscriptionAddress(
            targetModID, targetVarID, subModID, subVarID);
        Node node = Node(targetModID, 0);
        Variable& var = node.createVariable(targetVarID, "var", "scalar", 0.0F);
        TEST_ASSERT_EQUAL(var.subscribeFromModuleID,   0);
        TEST_ASSERT_EQUAL(var.subscribeFromVariableID, 0);
        node.setReceivedMessage(m1);
        node.run(0);
        TEST_ASSERT_EQUAL(var.subscribeFromModuleID,   subModID);
        TEST_ASSERT_EQUAL(var.subscribeFromVariableID, subVarID);
        TEST_ASSERT(var.isSubscribed);
    }
}

void test_modrob_messages_change_broadcast_change_var()
{
    for(auto i=0; i < numberOfTestIteration; i++){
        uint16_t targetModID = rand16Bit(); // адрес модуля-подписчика
        uint8_t  targetVarID = rand8Bit();  // адрес переменной модуля-подписчика
        uint16_t subModID = rand16Bit();    // адрес модуля-публишера
        uint8_t  subVarID = rand8Bit();     // адрес переменной модуля-публишера
        uint8_t  typeID = rand8Bit();       
        float    targetValue = randFloat_minmax(-10000.0F, 10000.0F);  

        ModrobMessage subscribe = ModrobMessage::commandSetSubscriptionAddress(targetModID, targetVarID, subModID, subVarID);
        ModrobMessage broadcast = ModrobMessage::publishNewValue(typeID, subModID, subVarID, targetValue);

        Node node = Node(targetModID, 0);
        Variable& var = node.createVariable(targetVarID, "var", "scalar", 0.0F);

        TEST_ASSERT(var.subscribeFromModuleID   == 0);
        TEST_ASSERT(var.subscribeFromVariableID == 0);
        node.setReceivedMessage(subscribe);
        node.run(0);
        TEST_ASSERT(var.subscribeFromModuleID   == subModID);
        TEST_ASSERT(var.subscribeFromVariableID == subVarID);
        TEST_ASSERT(var.isSubscribed);
        node.setReceivedMessage(broadcast);
        node.run(100);
        TEST_ASSERT(var.isChanged);
        TEST_ASSERT_EQUAL(var.get(), targetValue);
        TEST_ASSERT(!var.isChanged);
    }
}

/** Массив из очередей, имитирующий CAN шину. Тип элементов очереди - Modrob сообщение. 
 * Каждая очередь соответствует одному узлу, всего таких узлов - 65536, поскольку module ID 
 * имеет размер в 16 бит, что соответствует такому числу узлов. 
 * Примечание: очередь фактически не ограничена по размеру, поэтому если её нужно ограничивать, 
 * делать это надо вручную. 
*/
array<queue<ModrobMessage>, 65536> canBus = {};

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

void test_modrob_multi_nodes_subscribe(){
    VirtualCanTransport::clear();

    VirtualCanTransport can0 = VirtualCanTransport(0);
    Node node0 = Node(0, 0);
    node0.setTransport(can0);
    auto& var0 = node0.createVariable(0, "var0", "scalar", 0.0);
    var0.set(10.54F);

    VirtualCanTransport can1 = VirtualCanTransport(1);
    Node node1 = Node(1, 1);
    node1.setTransport(can1);
    auto& var1 = node1.createVariable(1, "var1", "scalar", 0.0);

    node0.run(0);
    node1.run(0);

    TEST_ASSERT(var0.get() == 10.54F);
    TEST_ASSERT(var1.get() == 0);

    auto pubCmd = ModrobMessage::commandSetHertz(0, 0, 100);
    auto subCmd = ModrobMessage::commandSetSubscriptionAddress(1, 1, 0, 0);
    auto setCmd = ModrobMessage::commandSetValue(0, 0, 98.23);
    VirtualCanTransport::sendToBus(pubCmd);
    VirtualCanTransport::sendToBus(subCmd);
    // VirtualCanTransport::sendToBus(setCmd);

    node0.run(10001); // после первого запуска node0 получит команду публикации pubCmd
    node1.run(10001); // этот узел ничего не делает

    TEST_ASSERT(var0.hertz == 100);

    node0.run(20002); // узел публикует переменную var0 
    node1.run(20002); // после этого запуска node1 получит команду подписки subCmd

    TEST_ASSERT(var0.isPublished);
    TEST_ASSERT(var1.subscribeFromModuleID   == 0);
    TEST_ASSERT(var1.subscribeFromVariableID == 0);

    node0.run(30003); // узел публикует переменную var0 
    node1.run(30003); // и node1 получит сообщение от var0 и изменит переменную var1

    TEST_ASSERT(var0.isPublished);
    TEST_ASSERT(var0.get() == 10.54F);
    TEST_ASSERT(var1.get() == 10.54F);
}

void test_modrob_can_frames_receive()
{
    const size_t number_of_messages = 10;
    array<ModrobMessage, number_of_messages> messages = {};
    array<CAN_Message, number_of_messages> can_frames = {};
    modrob::system::fixed_circular_buffer<CAN_Message, 10> can_rx_buffer{};
    ModrobMessage msg_output = {};

    for (int i = 0; i < number_of_messages; i++)
    {
        unsigned int canID = 0;
        unsigned char data[8];
        messages[i].moduleID         = rand16Bit();
        messages[i].variableID       = rand8Bit();
        messages[i].messageType      = MessageType::Command;
        messages[i].value            = randFloat();
        messages[i].typeID           = rand8Bit();
        messages[i].targetVariableID = rand8Bit();
        messages[i].operationType    = OperationType ::SetSubscriptionAddress;
        messages[i].toCan(canID, data);
        
        can_frames[i] = CAN_Message(canID, data);
    }

    CAN_Message temp_msg = {};
    can_rx_buffer.emplace_back(can_frames[0]);
    can_rx_buffer.emplace_back(can_frames[1]);
    can_rx_buffer.emplace_back(can_frames[2]);

    temp_msg = can_rx_buffer.front();
    can_rx_buffer.pop_front();
    msg_output.fromCan(temp_msg.id, temp_msg.data);

    assert_messages_equal(messages[0], msg_output);
    TEST_ASSERT(can_rx_buffer.size() == 2);

    temp_msg = can_rx_buffer.front();
    can_rx_buffer.pop_front();
    msg_output.fromCan(temp_msg.id, temp_msg.data);

    assert_messages_equal(messages[1], msg_output);
    TEST_ASSERT(can_rx_buffer.size() == 1);

    temp_msg = can_rx_buffer.front();
    can_rx_buffer.pop_front();
    msg_output.fromCan(temp_msg.id, temp_msg.data);

    assert_messages_equal(messages[2], msg_output);
    TEST_ASSERT(can_rx_buffer.size() == 0);

}

enum class Result: uint8_t
{
    Ok      = 'O',
    Pending = 'P',
    Full    = 'F'
};

struct Serial_message
{
    Result state = Result::Pending;
    char buffer[512];
};


void test_lwrb_playground()
{
    lwrb_t buff;
    uint8_t buff_data[100] = {};
    uint8_t temp_buff[100] = {};
    TEST_ASSERT(temp_buff[0] = '0');
    lwrb_init(&buff, buff_data, sizeof(buff_data));
    // cout << "Number of bytes in buff before read: " << lwrb_get_full(&buff) << endl;
    // cout << "Number of bytes read: " << lwrb_read(&buff, temp_buff, sizeof(temp_buff)) << endl;

    system::fixed_circular_buffer<Serial_message, 10> buffer_of_commands;
    char command1[] = "Cosmic space commando base\n";
    char command2[] = "Shuriken Showdown\n";
    char command3[] = "Super Sonic Samurai\n";

    Serial_message msg1, msg2, msg3;

    memcpy(msg1.buffer, command1, sizeof(command1));
    memcpy(msg2.buffer, command2, sizeof(command2));

    buffer_of_commands.emplace_back(msg1);
    buffer_of_commands.emplace_back(msg2);
    TEST_ASSERT(buffer_of_commands.size() == 2);


    lwrb_write(&buff, command3, sizeof(command3));
    lwrb_read(&buff, temp_buff, 100);

    // cout << temp_buff << endl;
    memcpy(msg3.buffer, temp_buff, 100);
    cout << sizeof(msg3.buffer) << endl;
    memset(temp_buff, 0, sizeof(temp_buff));
    cout << temp_buff[0] << endl;
    TEST_ASSERT(temp_buff[0] = '0');
    // buffer_of_commands.emplace_back(msg3);

    // TEST_ASSERT(buffer_of_commands.size() == 3);

    // Serial_message temp_msg = buffer_of_commands.front();
    // cout << temp_msg.buffer << endl;

    // buffer_of_commands.pop_front();

    // TEST_ASSERT(buffer_of_commands.size() == 2);

    // temp_msg = buffer_of_commands.front();
    // cout << temp_msg.buffer << endl;

    // uint8_t limit_symbol;
    // uint8_t last_byte[1];
    // size_t n = lwrb_get_full(&buff);
    // cout << n << endl;
    // lwrb_peek(&buff, n-2, last_byte, 1);
    // cout << last_byte[0] << endl;
    // limit_symbol = '\n';
    // TEST_ASSERT(last_byte[0] == limit_symbol);
    // cout << last_byte[0] << endl;

}

void test_arduinojson()
{
    StaticJsonDocument<200> doc;
    char json[] =
      "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}fhgfhfghdfhg";
    // if (json[sizeof(json) - 1])
    cout << sizeof(json) << endl;
    cout << json[sizeof(json)-2] << endl;
    DeserializationError error = deserializeJson(doc, json);
    cout << error.c_str() << endl;
    const char* sensor = doc["sensor"];
    long time = doc["time"];
    double latitude = doc["data"][0];
    double longitude = doc["data"][1];
    cout << sensor << endl;
    cout << time << endl;

    doc.clear();
    doc["modID"] = 1;
    doc["varID"] = 2;
    doc["value"] = 23.43;
    doc["hertz"] = 1500;

    char serial_buffer[256] = "12345\n";
    cout << strlen(serial_buffer) << endl;
    cout << serial_buffer[strlen(serial_buffer)-2] << endl;
    cout << "Number of bytes written by JSON: " << serializeJson(doc, serial_buffer) << endl;
    cout << serial_buffer << endl;
    cout << "Number of bytes returned by strlen: " << strlen(serial_buffer) << endl;
    uint16_t hertz = 100;
}


void compose_hashID(int& id, uint16_t subModuleID, uint8_t subVariableID)
{
    id = ((subModuleID & 0xFFFFU) << 8U) | (subVariableID & 0xFF);
}

void decompose_hashID(int& id, uint16_t& subModuleID, uint8_t& subVariableID)
{
    subModuleID =  (id & 0xFFFF00U) >> 8U;
    subVariableID = id & 0x0000FFU;
}

void test_compose_decompose_hashID()
{
    uint16_t subModID_after;
    uint8_t  subVarID_after;
    for (auto i=0; i < numberOfTestIteration; i++)
    {
        uint16_t subModID = rand16Bit();
        uint8_t  subVarID = rand8Bit();
        int id = 0;
        compose_hashID(id, subModID, subVarID);
        decompose_hashID(id, subModID_after, subVarID_after);
        TEST_ASSERT_EQUAL(subModID, subModID_after);
        TEST_ASSERT_EQUAL(subVarID, subVarID_after);
    }
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

    void send(const VariableInfo& variable, AggregatorMessage type) override
    {
        StaticJsonDocument<256> doc = {};
        doc["modID"] = variable.moduleID;
        doc["varID"] = variable.variableID;
        doc["value"] = round2(variable.value);
        doc["hertz"] = variable.hertz;
        serializeJson(doc, serial_buffer);
        serial_buffer[strlen(serial_buffer)] = '\n';
        cout << serial_buffer;
        memset(serial_buffer, 0, sizeof(serial_buffer));
    }
    bool receive(VariableInfo& result) override
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
    // VariableInfo& var_info1 = broker.find_publisher(1, 1, check_key);
    // TEST_ASSERT_EQUAL(var_info1.moduleID, 1);
    // TEST_ASSERT_EQUAL(var_info1.value, 0);
    // VariableInfo& var_info2 = broker.find_publisher(2, 2, check_key);
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

    // node1.run(3);
    // node2.run(3);
    // broker.run(3);

    // TEST_ASSERT_EQUAL(var1.value, 12.12);
    // var_info1 = broker.find_publisher(1, 1, check_key);
    // TEST_ASSERT_EQUAL(var_info1.moduleID, 1);
    // TEST_ASSERT_EQUAL(var_info1.value, 0);
    // var_info2 = broker.find_publisher(2, 2, check_key);
    // TEST_ASSERT_EQUAL(var_info2.moduleID, 2);
    // TEST_ASSERT_EQUAL(var_info2.value, 0);

    // node1.run(4);
    // node2.run(4);
    // broker.run(4);

    // TEST_ASSERT_EQUAL(var1.value, 12.12);
    // TEST_ASSERT_EQUAL(var2.value, 32.4);
    // var_info1 = broker.find_publisher(1, 1, check_key);
    // TEST_ASSERT_EQUAL(var_info1.moduleID, 1);
    // TEST_ASSERT_EQUAL(var_info1.value, 0);
    // var_info2 = broker.find_publisher(2, 2, check_key);
    // TEST_ASSERT_EQUAL(var_info2.moduleID, 2);
    // TEST_ASSERT_EQUAL(var_info2.value, 0);
    
    // node1.run(5001);
    // node2.run(5001);
    // broker.run(5001);

    // var_info1 = broker.find_publisher(1, 1, check_key);
    // TEST_ASSERT_EQUAL(var_info1.moduleID, 1);
    // var_info2 = broker.find_publisher(2, 2, check_key);
    // TEST_ASSERT_EQUAL(var_info2.value, 32.4);

    // node1.run(10002);
    // node2.run(10002);
    // broker.run(10002);

    


    for (long t = 0; t < 21000; t++)
    {
        node1.run(t);
        node2.run(t);
        broker.run(t);
    }

}


int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_size_of_structures);
    // RUN_TEST(test_modrob_adding_variables);
    // RUN_TEST(test_modrob_get_set_var);
    // RUN_TEST(test_modrob_messages_from_to);
    // RUN_TEST(test_modrob_messages_from_to_value);
    // RUN_TEST(test_modrob_messages_set_get_address);
    // RUN_TEST(test_modrob_messages_change_var);
    // RUN_TEST(test_modrob_messages_change_hertz);
    // RUN_TEST(test_modrob_messages_change_subscription_address);
    // RUN_TEST(test_modrob_messages_change_broadcast_change_var);
    // RUN_TEST(test_modrob_multi_nodes_subscribe);
    // RUN_TEST(test_modrob_can_frames_receive);
    // RUN_TEST(test_lwrb_playground);
    RUN_TEST(test_arduinojson);
    // RUN_TEST(test_compose_decompose_hashID);
    RUN_TEST(test_modrob_hash_tables);
    RUN_TEST(test_modrob_aggregator);
    UNITY_END();

}