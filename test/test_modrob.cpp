#include "unity.h"
#include <modrob.hpp>
#include <string>
#include <iostream>
#include <queue>
#include <vector>
#include <random>
#include <cstdarg>
#include <array>
#include "string.h"
#include <bitset>
#include "lwrb.h"
#include "../lib/ArduinoJson.h"

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
template<typename T>
unsigned count_non_val(T first, T last, const decltype(*first)& val){
    unsigned result=0;
    for(T i=first; i!=last; ++i)
        if(*i!=val)
            ++result;
    return result;
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

    char serial_buffer[256] = {"abcdefgh\n"};
    cout << strlen(serial_buffer) << endl;
    cout << serial_buffer[strlen(serial_buffer)] << endl;
    TEST_ASSERT(serial_buffer[strlen(serial_buffer) - 1] == '\n');
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_size_of_structures);
    RUN_TEST(test_modrob_adding_variables);
    RUN_TEST(test_modrob_get_set_var);
    RUN_TEST(test_modrob_messages_from_to);
    RUN_TEST(test_modrob_messages_from_to_value);
    RUN_TEST(test_modrob_messages_set_get_address);
    RUN_TEST(test_modrob_messages_change_var);
    RUN_TEST(test_modrob_messages_change_hertz);
    RUN_TEST(test_modrob_messages_change_subscription_address);
    RUN_TEST(test_modrob_messages_change_broadcast_change_var);
    // RUN_TEST(test_modrob_multi_nodes_subscribe);
    // RUN_TEST(test_modrob_can_frames_receive);
    // RUN_TEST(test_lwrb_playground);
    RUN_TEST(test_arduinojson);
    UNITY_END();

}