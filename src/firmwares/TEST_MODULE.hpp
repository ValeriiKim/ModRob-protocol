#ifndef MODROB_TEST_MODULE_HPP
#define MODROB_TEST_MODULE_HPP

// Главная тестовая прошивка для отладки различной периферии и протокола

// Библиотеки микроконтроллера
#include "board_config.h"
#include "usart_dma_config.h"
#include "can_config.h"
#include "timer2.h"


// Библиотеки протокола
#include "modrob_can.hpp"
#include "modrob_usart_logger.hpp"


// Макросы для дебага
#define GREEN_LED_ON    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF   SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON      SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF     SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);

CanModrobTransport can_transport;
// SerialCommander commander;

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


	Node node(MODULE_ID, MODULE_TYPE_ID);
	node.setTransport(can_transport);

	auto& var0 = node.createVariable(0, "var0", "scalar", 0.0);


	while(1) 
	{
		node.run(timer2::get_micros());
		// commander.run(node);

		// usart2::usart_send_float(var0.get(), 1);

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

// обработчик прерывания от памяти к периферии (USART2-TX) для дебага
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
		// commander.serial_commander_IRQHandler();
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





#endif //MODROB_TEST_HPP
