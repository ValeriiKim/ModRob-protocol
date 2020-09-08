#ifndef MODROB_CAN_HPP
#define MODROB_CAN_HPP

#include "modrob.hpp"
#include "can_config.h"
#include "usart_dma_config.h"

using namespace modrob;
using namespace can_bus;

#define USE_CANRX_INTERRUPT true

class CanModrobTransport: public ModrobTransport 
{
    CAN_Message received_can_msg    = {};
    CAN_Message transmitted_can_msg = {};

public:
    system::fixed_circular_buffer<CAN_Message, 10> rx_buffer{};

/** Инициализация bxCAN МК STM32F103C8T6 - скорость фиксированная 
 * и составляет 1 Мбит/с, настраиваются пины PB8 (RX) и PB9 (TX)
 * @param canRX_interrupt флаг, позволяющий включить прерывания по приёму (настройка 
 * в can_setup())
*/
    CanModrobTransport()
    {
        // can_setup(canRX_interrupt);
        // can_start();
    }

    void init(bool canRX_interrupt)
    {
        can_setup(canRX_interrupt);
        can_start();
    }

    void send(const ModrobMessage& msg) override 
    {
        unsigned int canID    = 0;
        unsigned char data[8] = {};
        msg.toCan(canID, data);     // конвертируем ModrobMessage в формат CAN сообщения
        transmitted_can_msg = CAN_Message(canID, data); // формируем CAN сообщение
        transmitted_can_msg.format = EXTENDED_FORMAT;
        bool is_write_ok = can_write(&transmitted_can_msg);
        if (is_write_ok != 1)
        {
            usart2::usart_send_string("CAN writing isn't ok!");
        }
    }

    bool receive(ModrobMessage& result) override
    {
        #if USE_CANRX_INTERRUPT == false
        if (can_read(&received_can_msg) == 1)
        {
            // Формируем Modrob сообщение из CAN фрейма
            result.fromCan(received_can_msg.id, received_can_msg.data);
            return true;
        }
        else
        {
            return false;
        }
        #else
        if (rx_buffer.not_empty())
        {
            CAN_Message temp_msg = {};
            temp_msg = rx_buffer.front();
            rx_buffer.pop_front();
            result.fromCan(temp_msg.id, temp_msg.data);
            return true;
        }
        else
        {
            return false;
        }
        #endif
    }

    void can_modrob_IRQHandler()
    {
        can_read(&received_can_msg);
        rx_buffer.emplace_back(received_can_msg);
    }

};

























#endif