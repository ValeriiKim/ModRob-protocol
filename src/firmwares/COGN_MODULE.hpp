#ifndef MODROB_COGN_MODULE_HPP
#define MODROB_COGN_MODULE_HPP

// Главная тестовая прошивка для отладки различной периферии и протокола

// Библиотеки микроконтроллера
#include "board_config.h"
#include "usart_dma_config.h"
#include "can_config.h"
#include "timer2.h"


// Библиотеки протокола
#include "modrob_can.hpp"
#include "modrob_usart_logger.hpp"
#include "modrob_serial_command.hpp"
#include "modrob_serial_transport.hpp"
#include "modrob_aggregator.hpp"


// Макросы для дебага
#define GREEN_LED_ON    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF   SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON      SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF     SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);

SerialJSONTransport serial_transport;
CanModrobTransport can_transport;
SerialCommander commander;

int main() {
	board::system_config();
	board::sys_tick_init();
	board::gpio_setup();
	usart2::usart2_init();
	timer2::tim2_setup();
	timer2::tim2_start();

	can_transport.init(true);
	usart2::usart_send_string("CAN controller initialized and started\n");

    GREEN_LED_OFF;
	TESTPIN_OFF;


	Aggregator node(MODULE_ID, MODULE_TYPE_ID);
	node.setUpperTransport(serial_transport);
	node.setTransport(can_transport);


	while(1) 
	{
		// TESTPIN_ON;
		node.run(timer2::get_micros());
		// TESTPIN_OFF;
		commander.run(node);
		// usart2::usart_send_float(12.23, 1);
		// usart2::usart_send_string("12345678\n");

	}

	return 0;
}

/* Нужно добавлять для обработчиков прерываний (пока непонятно для всех
или для только SysTick) обёртку, которая позволяет воспринимать 
участок кода как код языка С (НЕ С++). Без этого функции обработчиков
прерываний не хотят работать  */
extern "C" 
{
    void SysTick_Handler(void) 
    {
    	board::systick_delay_handler();
    }
}

// обработчик прерывания от памяти к периферии (USART2-TX)
extern "C" 
{
	void DMA1_Channel7_IRQHandler(void)
	{
		usart2::transfer_handler_irq();
	}
}


extern "C"
{
	/* Обработка прерывания по переносу данных из сдвигового регистра в USART_DR при приёме данных по RX */
	void USART2_IRQHandler(void) 
	{
		// usart2::receive_handler_irq();
		commander.serial_commander_IRQHandler();
	}
}


/** Обработчик прерывания по приёму CAN фреймов, прерывание происходит, когда 
 * в FIFO0 появляется хотя бы одно сообщение. Здесь происходит чтение сообщения */ 
extern "C"
{
	void USB_LP_CAN1_RX0_IRQHandler(void)
	{
		if (CAN1->RF0R & CAN_RF0R_FMP0) // FMP0 > 0? 
		{
			can_transport.can_modrob_IRQHandler();
		}
	}
}





#endif 
