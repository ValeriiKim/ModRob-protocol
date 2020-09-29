#ifndef USART_DMA_CONFIG_H
#define USART_DMA_CONFIG_H

#include "stm32f1xx.h"
#include "lwrb.h"
#include "cmath"
#include "printf.h"

#define GREEN_LED_ON    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF   SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
#define TESTPIN_ON      SET_BIT(GPIOA->BSRR, GPIO_BSRR_BR9);
#define TESTPIN_OFF     SET_BIT(GPIOA->BSRR, GPIO_BSRR_BS9);

namespace usart2
{
	static volatile uint32_t tmpreg;
	// Переменные для работы с буфером UART (DMA TX и приём RX)
	// Набор переменных для буфера передачи TX
	static lwrb_t usart_dma_tx_buff;
	static uint8_t usart_dma_tx_buff_data[512];
	static size_t usart_dma_tx_len;
	// Набор переменных для буфера приёма RX
	static lwrb_t usart_rx_ringbuf;
	static uint8_t usart_rx_ringbuf_data[512];
	// Вспомогательный буфер для получения байтов после фиксации лимитирующего символа
	uint8_t temp_buffer[512];
	// Лимитирующий символ
	uint8_t limit_symbol;
	// Флаг, активирующий отслеживание лимитирующего символа при приёме
	bool limit_check_flag = false;
	// Флаг, показывающий, что достигнут лимитирующий символ и строка скопирована в буфер
	bool rx_data_copied = false;

	/* Инициализация DMA1 для USART2, функция должна быть вызвана внутри
функции USART2_Init перед настройкой управляющих регистров. На данный момент
решено использовать DMA только для передачи (TX), поскольку согласно концепции не
планируется отправлять от компьютера большой поток данных вниз на модуль. Достаточно будет
отправлять небольшие команды или установки каких-то переменных, адресов и т.д. */
	static void dma1_init_for_usart2()
	{
		// включаем тактирование контроллера DMA1
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		tmpreg = (RCC->AHBENR & RCC_AHBENR_DMA1EN);
		// устанавливаем приоритет канала DMA (пока точно неясно какой нужен)
		NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
		NVIC_EnableIRQ(DMA1_Channel7_IRQn);

		// На всякий случай выключаем канал 7
		DMA1_Channel7->CCR &= ~DMA_CCR_EN;

		/* 
	Настройка USART2_TX на передачу "наверх" к ПК. Пока что используется обычный фиксированный
	буфер для передачи*/
		// Настройка направления передачи - Memory to Peripheral
		DMA1_Channel7->CCR &= ~DMA_CCR_MEM2MEM;
		DMA1_Channel7->CCR |= DMA_CCR_DIR;
		// установка уровня приоритета - Low (пока непонятно какой нужен)
		DMA1_Channel7->CCR &= ~DMA_CCR_PL;
		// Режим передачи NORMAL - Circular mode disabled
		DMA1_Channel7->CCR &= ~DMA_CCR_CIRC;
		//Set peripheral to NO increment mode DMA_CCR_PINC = 0
		DMA1_Channel7->CCR &= ~DMA_CCR_PINC;
		//Set memory increment mode
		DMA1_Channel7->CCR |= DMA_CCR_MINC;
		// Set peripheral data width - устанавливаем ширину данных с периферии
		// нужно 8 бит, поскольку USART получает 8 бит полезных данных
		DMA1_Channel7->CCR &= ~(DMA_CCR_PSIZE_1 | DMA_CCR_PSIZE_0);
		// Set memory data width - устанавливаем ширину данных в памяти
		// здесь тоже нужно 8 бит для согласованности
		DMA1_Channel7->CCR &= ~(DMA_CCR_MSIZE_1 | DMA_CCR_MSIZE_0);

		// Конфигурация адресов источника и приёмника для TX (write)
		DMA1_Channel7->CPAR = (uint32_t) & (USART2->DR); // адрес регистра приёмника

		// Enable Channel 7 Transfer complete interrupt and Transfer error interrupt
		DMA1_Channel7->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

		// Clear Channel 7 global interrupt flag, transfer complete flag and transfer error flag
		DMA1->IFCR = DMA_IFCR_CGIF7;
		DMA1->IFCR = DMA_IFCR_CTCIF7;
		DMA1->IFCR = DMA_IFCR_CTEIF7;
	}

/** Инициализация USART2 - все настройки стандартные: асинхронный режим, фиксированная 
* скорость 115200 бит/с, длина слова 8 бит, 1 стоп бит, контроля чётности нет,
* есть приём и передача, порты PA2 - TX, PA3 - RX
 */
	void usart2_init()
	{
		// инициализация USART2
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // включаем тактирование USART2
		tmpreg = RCC->APB1ENR & RCC_APB1ENR_USART2EN;
		/* Настройка портов - сначала нужно сбросить все настройки, 
	настройка порта PA2 (TX) на выход с максимальной скоростью,
	режим - alternate function push-pull.
	настройка порта PA3 (RX) на вход с режимом floating input */
		GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
		GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);

		GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0; // 0b11 50MHz output
		GPIOA->CRL |= GPIO_CRL_CNF2_1;					   // PA2 (TX): output 50MHz - Alt-function Push-pull

		GPIOA->CRL |= GPIO_CRL_CNF3_0; // PA3 (RX): input floating

		dma1_init_for_usart2();

		/* настраиваем регистры USART2: асинхронный режим, включаем приём и передачу,
	устанавливаем скорость передачи - baudrate (пока что фиксированная
	скорость - 115200 бит/с), длина слова - 8 бит, 1 стоп бит */
		if ((USART2->CR1 & USART_CR1_UE) != USART_CR1_UE) // проверяем, вдруг уже включен USART
		{
			USART2->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
			USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
		}
		USART2->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
		USART2->CR3 &= ~(USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL);
		USART2->BRR = 0x138; // рассчитывалось по даташиту, равно 19.53

		/* Включаем прерывания RXNE, которые генерируются если ORE=1 или RXNE=1 
	в регистре статуса USART_SR, т.е соответственно при переполнении (ORE) и когда
	принятые данные готовы к чтению (RXNE) */
		USART2->CR1 |= USART_CR1_RXNEIE;

		//Enable DMA Mode for transmission (DMAT) only
		USART2->CR3 |= USART_CR3_DMAT;

		// На всякий случай очищаем регистр передачи данных USART2
		USART2->DR = 0x00;
		USART2->SR |= USART_SR_TC;

		// разрешим глобальные прерывания USART2 для приёма данных по линии RX
		NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 1));
		NVIC_EnableIRQ(USART2_IRQn);

		// Включаем USART2
		USART2->CR1 |= USART_CR1_UE; // enable USART2

		lwrb_init(&usart_dma_tx_buff, usart_dma_tx_buff_data, sizeof(usart_dma_tx_buff_data));
		lwrb_init(&usart_rx_ringbuf, usart_rx_ringbuf_data, sizeof(usart_rx_ringbuf_data));
	}

/* Отправка байт по USART2 с помощью DMA с использованием кольцевого буфера  
Details here: https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
 */
	inline uint8_t usart_start_tx_dma_transfer()
	{
		uint32_t old_primask;
		uint8_t start_flag = 0;

		old_primask = __get_PRIMASK();
		__disable_irq();

		// проверяем, что передача не активна в настоящий момент
		if (usart_dma_tx_len == 0)
		{
			// проверяем, есть ли что отправлять
			usart_dma_tx_len = lwrb_get_linear_block_read_length(&usart_dma_tx_buff);
			if (usart_dma_tx_len > 0)
			{
				// создаём указатель на первый элемент в линейном блоке буфера на передачу
				void *ptr = lwrb_get_linear_block_read_address(&usart_dma_tx_buff);
				// Выключаем 7 канал DMA
				DMA1_Channel7->CCR &= ~DMA_CCR_EN;
				DMA1->IFCR = DMA_IFCR_CTCIF7;
				DMA1->IFCR = DMA_IFCR_CGIF7;
				DMA1->IFCR = DMA_IFCR_CTCIF7;
				DMA1->IFCR = DMA_IFCR_CTEIF7;
				// Начинаем DMA передачу
				// обнуляем число байт, которое следует отправить и пишем новое число
				DMA1_Channel7->CNDTR &= ~DMA_CNDTR_NDT;
				DMA1_Channel7->CNDTR = usart_dma_tx_len;
				// устанавливаем адрес памяти для передачи
				DMA1_Channel7->CMAR = (uint32_t)(ptr); // !тут static_cast не работает!
				DMA1_Channel7->CCR |= DMA_CCR_EN;
				start_flag = 1;
			}
		}
		__set_PRIMASK(old_primask);
		return start_flag;
	}

/** Функция для включения отслеживания лимитирующего символа (обычно \n) при приёме байт. 
 *  Эта функция активирует выполнение условия в обработчике прерывания, которое переносит полученные байты 
 * из главного кольцевого буфера во временный буфер.
 */
    void activate_check_of_rxlimit(const unsigned char limit)
	{
		limit_check_flag = true;
		limit_symbol = limit;
	}

/** Отправка константной строки по USART2 с помощью DMA, однако несмотря на это DMA, команда 
 * блокирующая, т.е. она ждёт, пока передаваемая строка полностью не отправится
*@param str указатель на строку  
 */
	void usart_send_string(const char *str)
	{
		// Записываем новые данные в буфер, только если предыдущая передача по USART уже закончилась
		while((USART2->SR & USART_SR_TC) != USART_SR_TC) {}
		// if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
		// {
		USART2->SR &= ~USART_SR_TC;
		size_t len = strlen(str);
		if (lwrb_get_free(&usart_dma_tx_buff) >= len){
			lwrb_write(&usart_dma_tx_buff, str, strlen(str));
		    usart_start_tx_dma_transfer();
		}
		
		// }
	}

/* Функция для передачи полученных по RX байт в TX (режим loop-back)
 */
	void usart_echo()
	{
		uint8_t temp_buffer[128];
		// Ждём пока предыдущая передача по TX не закончится (пока не установится флаг)
		while((USART2->SR & USART_SR_TC) != USART_SR_TC) {}
		// if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
		// {
			// USART2->SR &= ~USART_SR_TC;
			// получаем число байт линейной части буфера, которые можно читать
			// size_t len = lwrb_get_linear_block_read_length(&usart_rx_ringbuf);
			size_t len = lwrb_get_full(&usart_rx_ringbuf);
			// читаем байты из буфера RX и переписываем их в temp_buffer
			lwrb_read(&usart_rx_ringbuf, temp_buffer, len);
			// пишем эти байты в буфер для отправки через DMA 
			lwrb_write(&usart_dma_tx_buff, temp_buffer, len);
			usart_start_tx_dma_transfer();
		// }
	}

/** Отправка целого числа (int) по USART2 с помощью DMA, однако команда блокирующая, но в том смысле, 
 * что необходимо ожидать, пока завершится передача переданного значения. Это необходимо, чтобы 
 * какие-либо передаваемые данные не "перекрывали" другие
*@param value значение integer, которое нужно печатать
*@param newline логическое значение, позволяющее печатать число с новой строки, если
newline равно true
*@return целое число - результат выполнения программы, если 0, передача в буфер
 DMA успешна, если -1, не хватило памяти буфера для копирования
 */
	uint8_t usart_send_int(int value, bool newline)
	{
		int result; 
		char temp_buffer[128];
		char new_line[] = "\n";
		size_t newline_size = 0;

		result = snprintf_(temp_buffer, sizeof(temp_buffer), "%i", value);
		if (result < 0) {
			return -1; // копирование в буфер прошло неудачно, это ошибка
		}
		if (newline) // если хотим каждый символ печатать с новой строки
		{
			strcat(temp_buffer, new_line); // добавляем символ \n
			newline_size = 1; // указываем сколько нужно ещё байт отправить
		}

		// Записываем новые данные в буфер, только если предыдущая передача по USART уже закончилась
		/* чтобы каждая функция в программе что-то печатала, нужно использовать
		этот блокирующий цикл */
		while ((USART2->SR & USART_SR_TC) != USART_SR_TC) {}
		// if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
		// {
		USART2->SR &= ~USART_SR_TC;
		lwrb_write(&usart_dma_tx_buff, temp_buffer, result + newline_size);
		usart_start_tx_dma_transfer();
		// }
		return 0;
	}

/** Отправка числа с плавающей точкой (float с 4 знаками после запятой) по USART2 с помощью DMA, однако команда блокирующая, но в том смысле, 
 * что необходимо ожидать, пока завершится передача переданного значения. Это необходимо, чтобы 
 * какие-либо передаваемые данные не "перекрывали" другие
*@param value значение float, которое нужно печатать
*@param newline логическое значение, позволяющее печатать число с новой строки, если
newline равно true
*@return целое число - результат выполнения программы, если 0, передача в буфер
 DMA успешна, если -1, не хватило памяти буфера для копирования
 */
	uint8_t usart_send_float(float value, bool newline)
	{
		int result;
		char temp_buffer[32];
		char new_line[] = "\n";
		size_t newline_size = 0;

		result = snprintf_(temp_buffer, sizeof(temp_buffer), "%.4f", value);
		if (result < 0) {
			return -1; // копирование в буфер прошло неудачно, это ошибка
		}
		if (newline) // если хотим каждый символ печатать с новой строки
		{
			strcat(temp_buffer, new_line); // добавляем символ \n
			newline_size = 1; // указываем сколько нужно ещё байт отправить
		}
		
		// Записываем новые данные в буфер, только если предыдущая передача по USART уже закончилась
		while ((USART2->SR & USART_SR_TC) != USART_SR_TC) {}
		USART2->SR &= ~USART_SR_TC;
		// if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
		// {
		lwrb_write(&usart_dma_tx_buff, temp_buffer, result + newline_size);
		usart_start_tx_dma_transfer();
		// }
		return 0;
	}


	/* Функция обрабатывающая прерывание по завершению передачи по DMA от памяти
к регистру данных USART2. Эта функция реализована конкретно для 7 канала DMA и
она должна быть вызвана в void DMA1_Channel7_IRQHandler(void)
 */
	inline void transfer_handler_irq()
	{
		if ((DMA1->ISR & DMA_ISR_TCIF7) == DMA_ISR_TCIF7) // сработало прерывание по окончанию передачи
		{
			DMA1->IFCR = DMA_IFCR_CTCIF7;
			lwrb_skip(&usart_dma_tx_buff, usart_dma_tx_len);
			usart_dma_tx_len = 0;
			usart_start_tx_dma_transfer();
		}
		else if ((DMA1->ISR & DMA_ISR_TEIF7) == DMA_ISR_TEIF7)
		{
			// выключаем оба канала DMA
			DMA1_Channel6->CCR &= ~DMA_CCR_EN;
			DMA1_Channel7->CCR &= ~DMA_CCR_EN;
		}
	}

/* Обработка прерывания по получению данных по RX. Функция должна быть вызвана
внутри функции void USART2_IRQHandler(void) 
 */
	inline void receive_handler_irq()
	{
		// произошло прерывание по получению данных: RXNE = 1
		if ((USART2->SR & USART_SR_RXNE) == USART_SR_RXNE)
		{
			uint8_t temp_byte[1];
			temp_byte[0] = static_cast<uint8_t>((USART2->DR & 0xFF)); // получаем байт из USART
			lwrb_write(&usart_rx_ringbuf, temp_byte, 1); // записываем байт в буфер приёма			
			// проверяем, получили ли мы лимитирующий символ строки, обычно это \n
			if ((temp_byte[0] == limit_symbol) && limit_check_flag && !rx_data_copied)
			{
			// переносим байты из главного буфера usart во временный буфер 
				lwrb_read(&usart_rx_ringbuf, temp_buffer, sizeof(temp_buffer));
			// устанавливаем флаг, что данные перенесены (сброс флага, после очистки буфера)
				rx_data_copied = true;
			}
		}

		// произошла ошибка по переполнению ORE = 1
		if ((USART2->SR & USART_SR_ORE) == USART_SR_ORE)
		{
			// просто очищаем этот флаг как указано в даташите
			(void)USART2->SR; // читаем регистр SR
			(void)USART2->DR; // и регистр DR
		}
	}

	void char_to_str(char* output, unsigned char* input, size_t size_of_input)
	{
		snprintf_(output, size_of_input, "%s", input); // %8s для CAN 
	}

	int receive_end(const unsigned char limit)
	{
		uint8_t last_byte[1];
		size_t n = lwrb_get_full(&usart_rx_ringbuf);
		lwrb_peek(&usart_rx_ringbuf, n-2, last_byte, 1);
		if (last_byte[0] == limit)
		{
			return 1;
		}
		return 0;
	}

	size_t get_string(char* buffer, size_t buffer_size)
	{
		return lwrb_read(&usart_rx_ringbuf, buffer, buffer_size);
	}

} // namespace usart2

#endif