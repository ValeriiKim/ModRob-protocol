#ifndef MODROB_USART_LOGGER_HPP
#define MODROB_USART_LOGGER_HPP

#include "usart_dma_config.h"

class Logger
{

private:
    static void send(const char* message, const char* level)
    {
        usart2::usart_send_string(message);
        usart2::usart_send_string(",  ");
        usart2::usart_send_string(level);
        usart2::usart_send_string("\n");
    }


public:
    static void send_int(int value)
    {
        usart2::usart_send_int(value, 1);
    }

    static void info(const char* message)
    {
        Logger::send(message, "INFO");
    }

    static void debug(const char* message)
    {
        if(LOG_LEVEL == 0)
        {
            Logger::send(message, "DEBUG");
        }
    }

    static void error(const char* message)
    {
        Logger::send(message, "ERROR");
    }

    static void warn(const char* message)
    {
        Logger::send(message, "WARN");
    }
};























#endif