#ifndef MODROB_AGGREGATOR_HPP
#define MODROB_AGGREGATOR_HPP

#include "modrob.hpp"
#include "../khash.h"

using namespace modrob;



class Aggregator : public Node
{
    // счётчик публикующихся переменных
    uint16_t pub_count;
    bool stop_all_log = false;

    KHASH_MAP_INIT_INT(pub, VariableInfo);
    khash_t(pub) *publishers = kh_init(pub);
    JSONTransport* upperTransport;
    AggregatorMessage msgType;

public:
    uint16_t cnt = 0;
    Aggregator(uint16_t address, uint8_t typeId): Node(address, typeId) {}

    void run(long time_us)
    {
        handleIncomingModRobMessages();
        log_variables(time_us);
    }

    void setUpperTransport(JSONTransport& newTransport)
    {
        upperTransport = &newTransport;
    }


/** Добавляет новую публикуемую переменную в хеш-таблицу публикующихся переменных
 * @param pub_modID идентификатор модуля, который публикует переменную
 * @param pub_varID идентификатор публикуемой переменной
 * @param pub_hertz частота публикуемой переменной
 */
    void add_new_publisher(uint16_t pub_modID, uint8_t pub_varID, uint16_t pub_hertz)
    {
        khint_t key;
        int id = 0;
        int ret;
        compose_hashID(id, pub_modID, pub_varID);
        key = kh_put(pub, publishers, id, &ret);
        if (ret == -1){
            return;
        }
        kh_value(publishers, key).moduleID = pub_modID;
        kh_value(publishers, key).variableID = pub_varID;
        kh_value(publishers, key).hertz = pub_hertz;
        kh_value(publishers, key).timeOfLastSend = 0;
        kh_value(publishers, key).value = 0;
        kh_value(publishers, key).isLogged = false;
        pub_count++;

    }


/** Поиск элемента хеш-таблицы по идентификатору
 * @param modID_find идентификатор модуля, который мы ищем
 * @param varID_find идентификатор переменной, которую мы ищем
 * @return ссылка на структуру найденной переменной
 */
    VariableInfo& find_publisher(uint16_t modID_find, uint8_t varID_find, khint_t& check_key)
    {
        int id_to_find = 0;
        khint_t key;
        compose_hashID(id_to_find, modID_find, varID_find);
        key = kh_get(pub, publishers, id_to_find);
        check_key = key;
        return kh_value(publishers, key);
    }

/** Удаление элемента хеш-таблицы по ID модуля и ID переменной 
 * @param modID_to_del идентификатор модуля, который мы хотим удалить из таблицы
 * @param varID_to_del идентификатор переменной, которую мы хотим удалить из таблицы
 */
    void remove_publisher(uint16_t modID_to_del, uint8_t varID_to_del)
    {
        int id_to_del;
        khint_t key;
        compose_hashID(id_to_del, modID_to_del, varID_to_del);
        key = kh_get(pub, publishers, id_to_del);
        if (key == kh_end(publishers)){
            return;
        }
        kh_del(pub, publishers, key);
        pub_count--;
    }

    void set_logging(uint16_t modID_to_log, uint8_t varID_to_log)
    {
        unsigned int check_key;
        VariableInfo& publisher_to_log = find_publisher(modID_to_log, varID_to_log, check_key);
        if (check_key == kh_end(publishers)) {return;}
        publisher_to_log.isLogged = true;
    }

    void reset_logging(uint16_t modID_to_log, uint8_t varID_to_log)
    {
        unsigned int check_key;
        VariableInfo& publisher_to_log = find_publisher(modID_to_log, varID_to_log, check_key);
        if (check_key == kh_end(publishers)) {return;}
        publisher_to_log.isLogged = false;
    }

    int get_publishers_num()
    {
        return kh_size(publishers);
    }

    void send_publishers_table()
    {
        stop_all_log = true;
        khint_t key;
        msgType = AggregatorMessage::VarTable;
        for (key = kh_begin(publishers); key != kh_end(publishers); ++key){
            if (kh_exist(publishers, key)){
                upperTransport->send(kh_value(publishers, key), msgType);
                }
        }
        stop_all_log = false;
    }

    // void send_publishers_info()
    // {
    //     uint8_t num = get_publishers_num();
    //     VariableInfo var;
    //     var.hertz = num;
    //     upperTransport->send(var);
    // }

private:

    void handleIncomingModRobMessages()
    {
        bool isReceived = Aggregator::transport->receive(incomingMsg) || isReceivedOutside;
        if(not isReceived) return;
        if((incomingMsg.messageType == MessageType::Publication) && (kh_size(publishers) != 0))
        {
            unsigned int check_key;
            VariableInfo& var_to_update = find_publisher(incomingMsg.moduleID, incomingMsg.variableID, check_key); 
            if (check_key == kh_end(publishers)) {
                return;
            }
            var_to_update.value = incomingMsg.getValue();
            var_to_update.isChanged = true;
        }
        isReceivedOutside = false;
    }

    void log_variables(long time)
    {
        if (stop_all_log) {return;}
        khint_t key;
        msgType = AggregatorMessage::Logging;
        for (key = kh_begin(publishers); key != kh_end(publishers); ++key){
            if (kh_exist(publishers, key) && kh_value(publishers, key).isLogged){
                if(kh_value(publishers, key).hertz > 0){
                    int microsDelay = 1000000 / kh_value(publishers, key).hertz;
                    // Logger::send_int(kh_value(publishers, key).timeOfLastSend);
                    if (time - kh_value(publishers, key).timeOfLastSend > microsDelay){
                        kh_value(publishers, key).timeOfLastSend = time;
                        upperTransport->send(kh_value(publishers, key), msgType);
                    }
                }
            }
        }
    }


    void compose_hashID(int& id, uint16_t ModuleID, uint8_t VariableID)
    {
        id = ((ModuleID & 0xFFFFU) << 8U) | (VariableID & 0xFF);
    }
    
    void decompose_hashID(int& id, uint16_t& ModuleID, uint8_t& VariableID)
    {
        ModuleID =  (id & 0xFFFF00U) >> 8U;
        VariableID = id & 0x0000FFU;
    }


};






#endif