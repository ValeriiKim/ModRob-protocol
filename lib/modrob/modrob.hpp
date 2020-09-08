
#ifndef MODROB_HPP
#define MODROB_HPP

#include <cstdint>
#include "fixed_vector.hpp"
//#include <cmath>
//#include <string>
//#include<iostream>

using namespace std;

namespace modrob {

    enum class MessageType: uint8_t {
        Command = 0,
        Publication = 1,
    };

    enum class OperationType: uint8_t {
        SetVariable = 0,
        SetFreqOfPublications = 1,
        SetSubscriptionAddress = 2,
    };


    /**
     * Сообщение отправляемое по основному траспорту протокола Modrob
     * (Чаще всего - CAN)
     */
    struct ModrobMessage{
        //CanMsgID
        uint16_t moduleID;
        uint8_t variableID;
        MessageType messageType;


        // Data
        uint32_t   value;
        uint8_t    targetVariableID; // TODO DELETE: переменная оказалась не нужна
        OperationType  operationType;
        uint8_t    typeID;

        void setValue(float newValue){
            static_assert(sizeof(float) == sizeof(uint32_t), "Wrong float type!");
            /** Type-punning https://stackoverflow.com/a/29867177 */
            union {
                float f;
                uint32_t i;
            } v;
            v.f = newValue;
            value = v.i;
        }

        uint32_t getHertz(){
            return value;
        }

        float getValue(){
            static_assert(sizeof(float) == sizeof(uint32_t), "Wrong float type!");
            /** Type-punning https://stackoverflow.com/a/29867177 */
            union {
                float f;
                uint32_t i;
            } v;
            v.i = value;
            return v.f;
        }

        void setTargetAddress(uint16_t modID, uint8_t varID){
            value = (static_cast<uint32_t>(modID) << 8U) | static_cast<uint32_t>(varID);
        }

        void getTargetAddress(uint16_t& modID, uint8_t& varID) const{
            modID = static_cast<uint16_t>((value >> 8U) & 0x0000FFFFU); 
            varID = static_cast<uint8_t>(value & 0x000000FFU);
        }

        /**
         * Преобразует Modrob сообщение в бинарный формат CAN
         * @param id идентификатор CAN сообщения, который формируется согласно протоколу
         * @param packet полезные данные CAN фрейма
         */
        void toCan(unsigned int& id, unsigned char (&packet)[8]) const{
            uint8_t addressTypeInt = static_cast<uint8_t>(messageType);
            /* CanID = moduleID (16 bit) + variableID (8 bit) + messageType (1 bit) */
            id = (moduleID << (8U + 1U)) | (variableID << 1U) | (addressTypeInt);

            /* Data = variable (32 bit) + targetVariableID (8 bit) + operationType (2 bit) + typeID (8 bit) */
            packet[0] =  value & 0x000000FFU;
            packet[1] = (value & 0x0000FF00U) >> 8U;
            packet[2] = (value & 0x00FF0000U) >> 16U;
            packet[3] = (value & 0xFF000000U) >> 24U;

            packet[4] = targetVariableID;
            packet[5] = static_cast<uint8_t >(operationType);
            packet[6] = typeID;
        }

        /**
         * Преобразует CAN сообщение в Modrob сообщение
         * @param id Идентификатор CAN сообщения, из которого формируется протокольное сообщение
         * @param packet Полезные данные CAN сообщения
         */
        void fromCan(const unsigned int& id, const uint8_t (&packet)[8]){
            /* moduleID (16 bit) + variableID (8 bit) + messageType (1 bit) */
            moduleID               = (id & 0x01FFFE00U) >> 9U;
            variableID             = (id & 0x000001FEU) >> 1U;
            uint8_t addressTypeInt = (id & 0x00000001U);
            messageType = static_cast<MessageType>(addressTypeInt);

            /* Data = variable (32 bit) + targetVariableID (8 bit) + operationType (2 bit) + typeID (8 bit) */
            uint8_t b0 = packet[0];
            uint8_t b1 = packet[1];
            uint8_t b2 = packet[2];
            uint8_t b3 = packet[3];

            value = (b3 << 24U) | (b2 << 16U) | (b1 << 8U) | b0;

            targetVariableID = packet[4];
            operationType    = static_cast<OperationType >(packet[5]);
            typeID           = packet[6];
        }

        static ModrobMessage commandSetValue(uint16_t toModule, uint8_t toVar, float newValue){
            ModrobMessage msg = {};
            msg.moduleID = toModule;
            msg.variableID = toVar;
            msg.targetVariableID = toVar;
            msg.setValue(newValue); // НАДО ПОПРАВИТЬ (newValue теперь float)
            msg.operationType   = OperationType::SetVariable;
            msg.messageType = MessageType::Command;
            return msg;
        }

        static ModrobMessage commandSetHertz(uint16_t toModule, uint8_t toVar, uint32_t hertz){
            ModrobMessage msg = {};
            msg.moduleID = toModule;
            msg.variableID = toVar;
            msg.targetVariableID = toVar;
            msg.value = hertz;
            msg.operationType   = OperationType::SetFreqOfPublications;
            msg.messageType = MessageType::Command;
            return msg;
        }

        static ModrobMessage commandSetSubscriptionAddress(uint16_t toModule, uint8_t toVar, uint16_t subModID, uint8_t subVarID){
            ModrobMessage msg = {};
            msg.moduleID = toModule;
            msg.variableID = toVar;
            msg.targetVariableID = toVar;
            msg.setTargetAddress(subModID, subVarID);
            msg.operationType   = OperationType::SetSubscriptionAddress;
            msg.messageType = MessageType::Command;
            return msg;
        }

        static ModrobMessage publishNewValue(uint8_t typeID, uint16_t fromModule, uint8_t fromVar, float newValue){
            ModrobMessage msg = {};
            msg.moduleID = fromModule;
            msg.variableID = fromVar;
            msg.setValue(newValue); // НАДО ПОПРАВИТЬ (newValue теперь float)
            msg.operationType = OperationType::SetVariable;
            msg.messageType = MessageType::Publication;
            msg.typeID = typeID;
            return msg;
        }

    private:

    };

    /**
     * Основной интерфейс транспорта для ModRob протокола
     * Возможные реализации: CAN, UDP или локальный (моделирование)
     */
    struct ModrobTransport {

        /**
         * Отправляет сообщение
         * @param msg сообщение, которое нужно отправить
         */
        virtual void send(const ModrobMessage& msg) = 0;

        /**
         * @param result ссылка на сруктуру куда будет записано новое сообщение
         * @return есть ли доступное сообщение (1) или нет (0)
         */
        virtual bool receive(ModrobMessage& result) = 0;
    };

    /**
     * Транспорт, который ничего не делает и ничего не получает
     */
    struct NoopTransport : public ModrobTransport {
        virtual void send(const ModrobMessage& msg) override {}
        virtual bool receive(ModrobMessage& result) override {
            return false;
        }
    };

    /**
     * Абстрактный класс мьютекса.
     */
    struct AbstractMutex{
        // TODO: разобраться, нужна блокировка
        virtual void lockMutex()=0;
        virtual void unlockMutex()=0;
    };

    /**
     * Переменная, значение которой будет:
     * - публиковаться на частоте [hertz]
     * - изменяться от команд ПК
     * - изменяться от других зависимых переменных модулей
     *
     * Содержит так же мета-информацию (описание, ед. изм и т.д.)
     * которая может понадобится в будущем
     */
    struct Variable{
        uint8_t   id;
        float value;
        float maxValue{0};
        float minValue{0};
//        string name;
//        string description;
//        string unit; // Measurement unit
        long hertz{0}; // частота публикации переменной
        bool isPersistent = false; // сохраняется ли переменная в постоянную память МК
        bool isReadOnly = false; // задается ли переменная только внешне (ПК или другие модули)
        bool isChanged = false; // поменялось ли значение с последннего чтения
        long timeOfLastSend = 0; // время, когда впоследний раз отправлялась переменная
        uint16_t subscribeFromModuleID{0}; // переменная, получающая ID модуля, на который подписан текущий модуль
        uint8_t  subscribeFromVariableID{0}; // переменная, получающая ID переменной, на которую подписан текущий модуль
        bool isSubscribed = false;
        bool isPublished = false; // for debug purpose

        Variable(uint8_t _id, float value): id(_id){
            set(value);
        }

        void set(float val){
            // TODO: мьютекс - проверка на запись
            value = val;
            isChanged = true;
        }

        float get(){
            // TODO: мьютекс - проверка на чтение
            isChanged = false;
            return value;
        }
    };


    /**
     * Программный узел, для коммуникации с другими модулями и ПК
     * Может быть только один на одном модуле. Узел может иметь определённое количество 
     * переменных (не более 127).
     */
    struct Node{

        constexpr static long maxVariables = 16;
        modrob::system::fixed_vector<Variable, maxVariables> variables{};
        uint16_t selfModuleID = 0;
        uint8_t  selfModuleTypeId = 0;
        ModrobMessage incomingMsg = {};
        ModrobMessage outgoingMsg = {};
        bool isReceivedOutside    = false;
        static NoopTransport defaultTransport;
        ModrobTransport* transport = &defaultTransport;

        explicit Node(uint16_t address, uint8_t typeId):
            selfModuleID(address), selfModuleTypeId(typeId){
        }

        /**
         * Объявляет новую переменную модуля
         * @param id Идентификатор переменной
         * @param name Название переменной
         * @param unit Единица измерения
         * @param defaultValue Значение переменной по-умолчанию
         * @return
         */
        Variable& createVariable(uint8_t id, const char* name, const char* unit, float defaultValue){
            // TODO: проверка на дублирующиеся ID (замена или игнор)
            // если есть ещё место для добавления новой переменной
            if(not variables.full()){
                Variable& var = variables.emplace_back(id, defaultValue); // добавляем новую переменную в вектор
                return var;              // возвращаем ссылку на новую переменную
            }else{
                return variables.back(); // если места нет, возвращаем последнюю переменную вектора
            }
        }

        void setTransport(ModrobTransport& newTransport){
            transport = &newTransport;
        }

        void setReceivedMessage(ModrobMessage& msg){
            isReceivedOutside = true;
            incomingMsg = msg;
        }

        /**
         * Вызывать с частотой не ниже 5 kHZ
         * - в главном бесконечном цикле (нельзя допускать блокирования в главном цикле)
         * - в прерываниях по таймеру
         * @param time количество миллисекунд с момента начала выполнения текущей программы на плате (функция millis())
         */
        void run(long time_us){
            handleIncomingModRobMessages(); // TODO: желательно вызывать на частоте > 5 kHZ
            publishAllVariables(time_us);
        }

    private:

        /**
         * Обработка входящих сообщений по выбранному транспорту
         */
        void handleIncomingModRobMessages(){
            // BUG: перезаписывание сообщения установлееного внешне
            bool isReceived = Node::transport->receive(incomingMsg) || isReceivedOutside;
            if(not isReceived) return;
            if(incomingMsg.messageType == MessageType::Command && incomingMsg.moduleID == selfModuleID){
                for(auto& var : variables){
                    if(incomingMsg.variableID == var.id){
                        switch(incomingMsg.operationType){
                            case OperationType::SetVariable:
                                var.value = incomingMsg.getValue();
                                var.isChanged = true;
                                break;
                            case OperationType::SetFreqOfPublications:
                                var.hertz = incomingMsg.value;
                                break;
                            case OperationType::SetSubscriptionAddress:
                                uint16_t modID = 0;
                                uint8_t  varID = 0;
                                incomingMsg.getTargetAddress(modID, varID);
                                var.subscribeFromModuleID   = modID;
                                var.subscribeFromVariableID = varID;
                                var.isSubscribed = true;
                                break;
                        }
                    }
                }
            }
            if(incomingMsg.messageType == MessageType::Publication){
                for(auto& var: variables){
                    bool isModuleIDMatch = incomingMsg.moduleID   == var.subscribeFromModuleID;
                    bool isVariableID    = incomingMsg.variableID == var.subscribeFromVariableID;
                    if(isModuleIDMatch && isVariableID && var.isSubscribed){
                        var.value = incomingMsg.getValue();
                        var.isChanged = true;
                    }
                }
            }
            isReceivedOutside = false;
        }

        /**
         * Публикация всех переменных с учетом их частоты публикации, может 
         * базироваться как на программном, так и на хардварном таймере
         */
        void publishAllVariables(long time){
            for(auto& var: variables){
                if(var.hertz > 0){
                    int microsDelay = 1000000 / var.hertz ;
                    if(time - var.timeOfLastSend > microsDelay){ // software timer
                        var.timeOfLastSend = time;
                        outgoingMsg = ModrobMessage::publishNewValue(selfModuleTypeId, selfModuleID, var.id, var.value);
                        transport->send(outgoingMsg);
                        var.isPublished = true;
                    }
                }
            }
        }
    };
    NoopTransport Node::defaultTransport{};

    /**
     * сжатый формат для переменной
     * для агрегирования значений со всех модулей
     */
    struct VariableInfo{
        uint16_t moduleID;
        uint8_t  variableID;
        uint8_t  typeID;
        uint32_t value;
    };

}


#endif //MODROB_HPP
