#ifndef MODROB_SERIAL_TRANSPORT_HPP
#define MODROB_SERIAL_TRANSPORT_HPP


#include "modrob.hpp"
#include "../ArduinoJson.h"
#include "modrob_usart_logger.hpp"

constexpr size_t MAX_LEN_OF_SERIAL_MESSAGE = 256;

using namespace modrob;

class SerialJSONTransport : public JSONTransport
{
    char serial_buffer[MAX_LEN_OF_SERIAL_MESSAGE]{0};


public:
    SerialJSONTransport(){}

    void send(const VariableInfo &variable, AggregatorMessage type) override
    {
        // const size_t capacity = JSON_OBJECT_SIZE(4);
        StaticJsonDocument<256> doc;
        switch (type)
        {
        case AggregatorMessage::Logging:
            doc["modID"] = variable.moduleID;
            doc["varID"] = variable.variableID;
            doc["value"] = round2(variable.value);
            serializeJson(doc, serial_buffer);
            serial_buffer[strlen(serial_buffer)] = '\n';
            usart2::usart_send_string(serial_buffer);
            break;

        case AggregatorMessage::VarTable:
            doc["Table"] = "Publishers";
            doc["modID"] = variable.moduleID;
            doc["varID"] = variable.variableID;
            doc["value"] = round2(variable.value);
            doc["hertz"] = variable.hertz;
            serializeJson(doc, serial_buffer);
            serial_buffer[strlen(serial_buffer)] = '\n';
            usart2::usart_send_string(serial_buffer);
            break;
        }

        memset(serial_buffer, 0, sizeof(serial_buffer));
    }

    bool receive(VariableInfo& result) override
    {
        return true;
    }

private:

    float round2(float value)
    {
        return static_cast<int>(value * 100 + 0.5) / 100.0;
    }
};





#endif