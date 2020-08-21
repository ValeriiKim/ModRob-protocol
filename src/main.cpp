// Главная тестовая прошивка для отладки различной периферии

// #include "stdio.h"
// Библиотеки микроконтроллера
#include "board_config.h"
#include "usart_dma_config.h"

// Библиотеки CanFestival
#include "canfestival.h"
#include "can_stm32.h"
#include "TestMaster.h"

// Различные вспомогательные библиотеки
// #include "lwrb.h"

#define ARRAY_LEN(x)  (sizeof(x) / sizeof((x)[0])) // определение числа элементов массива

#define GREEN_LED_ON    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF   SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON      SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF     SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);


int main() {

	board::system_config();
	board::sys_tick_init();
	board::gpio_setup();

    GREEN_LED_OFF;
	TESTPIN_OFF;

	usart2::usart2_init();
	// lwrb_init(&usart_dma_tx_buff, usart_dma_tx_buff_data, sizeof(usart_dma_tx_buff_data));
	// lwrb_init(&usart_rx_ringbuf, usart_rx_ringbuf_data, sizeof(usart_rx_ringbuf_data));

	// usart2::usart_send_string("Module successfully initialized\n");
	// usart2::usart_send_float(var);

	while(1) 
	{
		board::delay_ms(10);
		usart2::usart_send_string("Cosmic Space Commando Base\n");
		board::delay_ms(10);
		usart2::usart_send_string("Super Sonic Samurai\n");
		board::delay_ms(10);
		usart2::usart_send_string("Shuriken Showdown\n");
		board::delay_ms(10);
		usart2::usart_send_float(-123.321, true);
		board::delay_ms(10);
		usart2::usart_send_int(-700, true);
		// usart2::usart_send_int(8232323, true);
		// var++;
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
		TESTPIN_ON;
		usart2::transfer_handler_irq();
		TESTPIN_OFF;
	}
}



extern "C"
{
	/* Обработка прерывания по переносу данных из сдвигового регистра в USART_DR при приёме данных по RX */
	void USART2_IRQHandler(void) 
	{
		usart2::receive_handler_irq();
	}
}