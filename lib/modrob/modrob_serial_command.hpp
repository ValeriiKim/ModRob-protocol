#ifndef MODROB_SERIAL_COMMAND_HPP
#define MODROB_SERIAL_COMMAND_HPP

#include "modrob.hpp"
#include "../ArduinoJson.h"
#include "modrob_usart_logger.hpp"
#include "modrob_aggregator.hpp"

constexpr size_t MAX_LEN_OF_SERIAL_COMMAND = 256;
constexpr size_t MAX_BUFFERED_COMMANDS     = 10;    

#define COMMAND_GET_STATE "getState"

#define COMMAND_FIELD       "op"
#define COMMAND_SET_VAL     "setValue"
#define COMMAND_SET_PUB     "setPublication"
#define COMMAND_SET_SUB     "setSubscription"
#define COMMAND_START_LOG   "startLog"
#define COMMAND_STOP_LOG    "stopLog"
#define COMMAND_GET_PUB     "request_publishers_table"
#define COMMAND_RESET_PUB   "removePublication"
#define COMMAND_GET_NODES   "request_nodes_table"

#define MOD_FIELD "modID"
#define VAR_FIELD "varID"
#define VAL_FIELD "value"
#define FRQ_FIELD "frequency"
#define SMI_FIELD "subModID"
#define SVI_FIELD "subVarID"

using namespace modrob;

enum class Result: uint8_t
{
    Ok      = 'O', // Всё прочитано полностью
    Pending = 'P', // Конец строки ещё не получен
    Full    = 'F'  // Буфер заполнен
};

struct Serial_Message
{
    Result state = Result::Pending;
    char buffer[MAX_LEN_OF_SERIAL_COMMAND];
};


/* Класс для обработки комманд от ПК по сериалу
 * commands:
 * {"op": "getState"}
 * {"op": "setValue", "modID": N, "varID": N, "value": V}
 * {"op": "setPublication", "modID": N, "varID": N, "frequency": V}
 * {"op": "setSubscription", "modID": N, "varID": N, "subModID": V, "subVarID": V}
 * {"op": "startLog", "modID": N, "varID": V}
 * {"op": "stopLog", "modID": N, "varID": V}
 * {"op": "request_publishers_table"}
 * {"op": "request_nodes_table"}
}

 *
 * events:
 *
 * Examples:
 * {"op": "setValue", "modID": 1, "varID": 0, "value": 23.23}
 * {"op": "setPublication", "modID": 1, "varID": 0, "frequency": 200}
 * {"op": "startLog", "modID": 1, "varID": 0}
 * {"op": "stopLog", "modID": 1, "varID": 0}
 * {"op": "setSubscription", "modID": 0, "varID": 0, "subModID": 1, "subVarID": 1}
 */


class SerialCommander
{
private:
    // буфер, хранящий команду заканчивающуюся символом \n
    char serial_buffer[MAX_LEN_OF_SERIAL_COMMAND];
    // буфер сообщений, который хранит команды, присланные сверху
    system::fixed_circular_buffer<Serial_Message, MAX_BUFFERED_COMMANDS> buffer_of_commands;
    // временное сообщение, которое используется для записи в очередь или чтения
    Serial_Message temp_serial_msg;
    DeserializationError parse_error;
    StaticJsonDocument<256> doc = {};
    string op = "";
    ModrobMessage msg;

    void send_down(uint16_t modID, Aggregator &node, ModrobMessage &msg)
    {
        // если отправляем сообщение на сам модуль с Serial Commander
        if (modID == node.selfModuleID){
            // принимаем это сообщение здесь 
            node.setReceivedMessage(msg);
        }
        else {
            // иначе отправляем это сообщение по нижнему транспорту
            node.transport->send(msg);
        }
    }


    bool isCmdValid(){
        return doc.containsKey(COMMAND_FIELD) &&
               doc[COMMAND_FIELD].is<string>();
    }

    bool isValidSetValueCMD(){
        return doc.containsKey(MOD_FIELD) &&
               doc.containsKey(VAR_FIELD) &&
               doc.containsKey(VAL_FIELD) &&
               doc[MOD_FIELD].is<int>() &&
               doc[VAR_FIELD].is<int>() &&
               doc[VAL_FIELD].is<float>() &&
               doc[COMMAND_FIELD].as<string>() == COMMAND_SET_VAL;
    }

    bool isValidSetPublCMD(){
        return doc.containsKey(MOD_FIELD) &&
               doc.containsKey(VAR_FIELD) &&
               doc.containsKey(FRQ_FIELD) &&
               doc[MOD_FIELD].is<int>() &&
               doc[VAR_FIELD].is<int>() &&
               doc[FRQ_FIELD].is<int>() &&
               doc[COMMAND_FIELD].as<string>() == COMMAND_SET_PUB;
    }

    bool isValidSetSubCMD(){
        return doc.containsKey(MOD_FIELD) &&
               doc.containsKey(VAR_FIELD) &&
               doc.containsKey(SMI_FIELD) &&
               doc.containsKey(SVI_FIELD) &&
               doc[MOD_FIELD].is<int>() &&
               doc[VAR_FIELD].is<int>() &&
               doc[SMI_FIELD].is<int>() &&
               doc[SVI_FIELD].is<int>() &&
               doc[COMMAND_FIELD].as<string>() == COMMAND_SET_SUB;
    }

    bool isValidStartLogCMD(){
        return doc.containsKey(MOD_FIELD) &&
               doc.containsKey(VAR_FIELD) &&
               doc[COMMAND_FIELD].as<string>() == COMMAND_START_LOG;
    }

    bool isValidStopLogCMD(){
        return doc.containsKey(MOD_FIELD) &&
               doc.containsKey(VAR_FIELD) &&
               doc[COMMAND_FIELD].as<string>() == COMMAND_STOP_LOG;
    }

    bool isValidGetPubTable(){
        return doc[COMMAND_FIELD].as<string>() == COMMAND_GET_PUB;
    }

    bool isValidGetNodeTable(){
        return doc[COMMAND_FIELD].as<string>() == COMMAND_GET_NODES;
    }

public:
    SerialCommander()
    {
        usart2::activate_check_of_rxlimit('\n');
    }
    void run(Aggregator &node)
    {
        if (not buffer_of_commands.empty())
        {
            temp_serial_msg = buffer_of_commands.front();
            buffer_of_commands.pop_front();
            memcpy(serial_buffer, temp_serial_msg.buffer, MAX_LEN_OF_SERIAL_COMMAND);
            size_t buffer_size = strlen(serial_buffer);
            if ((serial_buffer[buffer_size - 2] == '}') | (serial_buffer[buffer_size - 2] == ']'))
            {
                parse_error = deserializeJson(doc, serial_buffer);
                if (parse_error == DeserializationError::Ok)
                {
                    if (isCmdValid())
                    {
                        op = doc[COMMAND_FIELD].as<string>();
                        if (op == COMMAND_GET_STATE)
                        {
                            // TODO: impl send state
                        }
                        else if (isValidSetValueCMD())
                        {
                            auto modID = doc[MOD_FIELD].as<int>();
                            auto varID = doc[VAR_FIELD].as<int>();
                            auto value = doc[VAL_FIELD].as<float>();
                            msg = ModrobMessage::commandSetValue(modID, varID, value);
                            send_down(modID, node, msg);
                        }
                        else if (isValidSetPublCMD())
                        {
                            auto modID = doc[MOD_FIELD].as<int>();
                            auto varID = doc[VAR_FIELD].as<int>();
                            auto hertz = doc[FRQ_FIELD].as<int>();
                            if (hertz > 0) {
                                node.add_new_publisher(modID, varID, hertz); // только если частота > 0
                                msg = ModrobMessage::commandSetHertz(modID, varID, hertz);
                                send_down(modID, node, msg);
                            }
                            if (hertz == 0) {
                                node.remove_publisher(modID, varID);
                                Logger::info("publisher removed");
                            }
                            if (hertz < 0) {Logger::warn("Frequency must be more than zero!");}
                        }
                        else if (isValidSetSubCMD())
                        {
                            auto modID = doc[MOD_FIELD].as<int>();
                            auto varID = doc[VAR_FIELD].as<int>();
                            auto subModID = doc[SMI_FIELD].as<int>();
                            auto subVarID = doc[SVI_FIELD].as<int>();
                            msg = ModrobMessage::commandSetSubscriptionAddress(modID, varID, subModID, subVarID);
                            send_down(modID, node, msg);
                        }
                        else if (isValidStartLogCMD())
                        {
                            auto modID = doc[MOD_FIELD].as<int>();
                            auto varID = doc[VAR_FIELD].as<int>();
                            node.set_logging(modID, varID);
                        }
                        else if (isValidStopLogCMD())
                        {
                            auto modID = doc[MOD_FIELD].as<int>();
                            auto varID = doc[VAR_FIELD].as<int>();
                            node.reset_logging(modID, varID);
                        }
                        else if (isValidGetPubTable())
                        {
                            node.send_publishers_table();
                        }
                        else if (isValidGetNodeTable())
                        {
                            node.send_node_table();
                        }
                        else {
                            Logger::warn("Not match command");
                        }
                    }
                    else {
                        Logger::error("Invalid command");
                    }
                }
                else {
                    Logger::error(parse_error.c_str());
                }
            }
            else {
                Logger::error("Incorrect JSON");
            }
        }
    }

    inline void serial_commander_IRQHandler()
    {
        usart2::receive_handler_irq();
        // если данные скопированы, начинаем перенос данных в буфер сообщений
        if (usart2::rx_data_copied)
        {
            // если буфер команд свободен, то
            if (not buffer_of_commands.full())
            {
                // копируем данные из временного буфера usart в буфер сообщения-команды (временное)
                memcpy(temp_serial_msg.buffer, usart2::temp_buffer, MAX_LEN_OF_SERIAL_COMMAND);
                // добавляем в конец очереди новое сообщение
                buffer_of_commands.emplace_back(temp_serial_msg);
                // мы готовы принимать новые команды, поэтому сбрасываем флаг
                usart2::rx_data_copied = false;
                // очищаем временный буфер, чтобы в него копировать новые данные
                memset(usart2::temp_buffer, 0, sizeof(usart2::temp_buffer));
            }
        }
    }
};

 










#endif