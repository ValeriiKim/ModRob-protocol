
// Библиотеки микроконтроллера
// #include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

// Библиотеки CanFestival
#include "canfestival.h"
#include "can_stm32.h"
#include "TestMaster.h"

#define ARRAY_LEN(x)  (sizeof(x) / sizeof((x)[0])) // определение числа элементов массива

#define SYSCLOCK 72000000U // максимальная частота контроллера

#define GREEN_LED_ON    SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
#define GREEN_LED_OFF   SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);

// Глобальные переменные
volatile uint32_t tmpreg;
volatile uint32_t SysTick_CNT = 0;

// Переменные для USART2 DMA
static uint8_t usart_rx_dma_buffer[64];
char usart_tx_dma_buffer[] = "Cosmic Space Commando Base\n";
uint8_t flag_rx = 0, flag_tx = 0;
char str1[30];

void Clock_DeInit()
{
	// Включаем в начале HSI (внутренний генератор 8 МГц)
	SET_BIT(RCC->CR, RCC_CR_HSION);
	// ждём пока не стабилизируется (определяется по флагу HSIRDY)
	while(READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET) {}
	// сбрасываем калибровку
	// TODO: понять почему здесь 0х80 и переписать командами SET
	MODIFY_REG(RCC->CR, RCC_CR_HSITRIM, 0x80U);
	// очищаем конфигурационный регистр
	CLEAR_REG(RCC->CFGR);
	// ждём очистки флага SWS
	while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RESET) {}
	// отключаем PLL
	CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
	// ждём пока контроллер не отпустит PLL (пока PLLRDY не станет равным 0)
	while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) != RESET) {}
	// выключаем HSE и его детектор тактового сигнала
	CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON);
	// ожидание пока HSE не отключится
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) != RESET) {}
	// сброс бита разрешающего использование внешнего генератора
	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);
	// сброс всех флагов прерываний от RCC
	SET_BIT(RCC->CSR, RCC_CSR_RMVF);
	// запрет всех прерываний от RCC
	CLEAR_REG(RCC->CIR);
}

void clock_init()
{
	// Настройка системной частоты на 72 МГц
	// Включим внешнее тактирование HSE и дождёмся его включения
	SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET) {}

	// настройка памяти FLASH, включается буфер предварительной выборки,
	// сначала он отключается, затем включается максимальная задержка
	CLEAR_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);

	// кажется в исходнике ошибка, FLASH_ACR_LATENCY_1 = 010 (то что нужно)
	// а FLASH_ACR_LATENCY_2, который везде указывается, на самом деле 100, такого режима нет 
    CLEAR_BIT(FLASH->ACR, FLASH_ACR_LATENCY);
	SET_BIT(FLASH->ACR, FLASH_ACR_LATENCY_1);

    // настраиваем делитель /1 AHB Prescaler
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_HPRE);
	SET_BIT(RCC->CFGR, RCC_CFGR_HPRE_DIV1);

	// настройка делителя /1 APB2 Prescaler
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE2);
	SET_BIT(RCC->CFGR, RCC_CFGR_PPRE2_DIV1);

	// настраиваем делитель /2 APB1 Prescaler
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PPRE1);
	SET_BIT(RCC->CFGR, RCC_CFGR_PPRE1_DIV2);

	// сбрасываем биты PLLSRC, PLLXTPRE, PLLMUL
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
	// выбираем HSE осциллятор как вход для PLL
	SET_BIT(RCC->CFGR, RCC_CFGR_PLLSRC);
	// настраиваем PLLMul на коэффициент 9
	SET_BIT(RCC->CFGR, RCC_CFGR_PLLMULL9);
    // включаем PLL и ждём пока он не заблокируется
	SET_BIT(RCC->CR, RCC_CR_PLLON);
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET) {}
	// выбираем PLL в качестве источника системного тактирования
	CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW); // сначала очищаем биты SW
	SET_BIT(RCC->CFGR, RCC_CFGR_SW_PLL);
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

}

/* Общая конфигурация системы: вызов функции настройки частоты тактирования 
на 72 МГц, включение тактирвоания AFIO, настройка портов для прошивания */
void system_config(void)
{ 
	clock_init();
	// включение тактирования AFIO
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
	// сбрасываем настройки SWD и JTAG
	CLEAR_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG);
	// и устанавливаем режим 010 - JTAG-DP Disabled and SW-DP Enabled
	SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

}

void sys_tick_init()
{
	CLEAR_BIT(SysTick->LOAD, SysTick_LOAD_RELOAD_Msk);
	// задаётся значение RELOAD, чтобы бы период был равен 1 мс
	// T = (RELOAD + 1)/freq, RELOAD = T*freq - 1 = 0.001*72000000 - 1
	SET_BIT(SysTick->LOAD, SYSCLOCK/1000 - 1);
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	// сброс счётчика
	CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	// настройка управляющего регистра: установка системной частоты, разрешение
	// прерываний и включение счётчика
	SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);   
}

void delay_ms(uint32_t ms)
{
    CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	SET_BIT(SysTick->VAL, SYSCLOCK/1000 - 1);
	SysTick_CNT = ms;
	while(SysTick_CNT) {}
}

void GPIO_setup()
{
	// включаем тактирование портов ввода/вывода А, В и С для отладки
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
	// ждём
	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
	// настраиваем порт С на выход push-pull с максимальной скоростью 50 МГц
	CLEAR_BIT(GPIOC->CRH, GPIO_CRH_CNF13);
	SET_BIT(GPIOC->CRH, GPIO_CRH_MODE13);
}


/* Инициализация DMA1 для USART2, функция должна быть вызвана внутри
функции USART2_Init перед настройкой управляющих регистров */
static void DMA1_init_for_USART2()
{
	// включаем тактирование контроллера DMA1
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	tmpreg = (RCC->AHBENR & RCC_AHBENR_DMA1EN);

    // DMA1_Channel6_IRQn interrupt init
	NVIC_SetPriority(DMA1_Channel6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	// GREEN_LED_OFF;
	// NVIC_SetPriority(DMA1_Channel7_IRQn, 0);
	// NVIC_SetPriority(DMA1_Channel6_IRQn, 0);
	// DMA1_Channel7_IRQn interrupt init
	NVIC_SetPriority(DMA1_Channel7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	// GREEN_LED_OFF;
	

	DMA1_Channel6->CCR &= ~DMA_CCR_EN;
	DMA1_Channel7->CCR &= ~DMA_CCR_EN;
	/* USART2 DMA Initialisation
	Настройка USART2_RX */
	// Настройка направления передачи - Peripheral to Memory
	DMA1_Channel6->CCR &= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
	// устанавливаем уровень приоритета - Low (пока непонятно какой нужен)
	DMA1_Channel6->CCR &= ~DMA_CCR_PL;
	// Режим передачи Circular mode 
	DMA1_Channel6->CCR |= DMA_CCR_CIRC;
	//Set peripheral to NO increment mode DMA_CCR_PINC = 0
	DMA1_Channel6->CCR &= ~DMA_CCR_PINC;
	//Set memory increment mode
	DMA1_Channel6->CCR |= DMA_CCR_MINC;
	// Set peripheral data width - устанавливаем ширину данных с периферии
	// нужно 8 бит, поскольку USART получает 8 бит полезных данных
	DMA1_Channel6->CCR &= ~(DMA_CCR_PSIZE_1 | DMA_CCR_PSIZE_0);
	// Set memory data width - устанавливаем ширину данных в памяти
	// здесь тоже нужно 8 бит для согласованности
	DMA1_Channel6->CCR &= ~(DMA_CCR_MSIZE_1 | DMA_CCR_MSIZE_0);

	// Конфигурация адресов источника и приёмника для RX (read)
	DMA1_Channel6->CPAR = (uint32_t)&(USART2->DR); // адрес регистра источника
	DMA1_Channel6->CMAR = (uint32_t)usart_rx_dma_buffer; // адрес регистра приёмника
	DMA1_Channel6->CNDTR &= ~DMA_CNDTR_NDT; // обнуляем число байтов, которые нужно отправить
	DMA1_Channel6->CNDTR = ARRAY_LEN(usart_rx_dma_buffer); // и пишем сколько реально планируется принять

	/* 
	Настройка USART2_TX */
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
	DMA1_Channel7->CPAR = (uint32_t)&(USART2->DR); // адрес регистра приёмника
	DMA1_Channel7->CMAR = (uint32_t)usart_tx_dma_buffer;

	// Enable Channel 6 Transfer complete interrupt and Transfer error interrupt
	DMA1_Channel6->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;
	// Enable Channel 7 Transfer complete interrupt and Transfer error interrupt
	DMA1_Channel7->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

	// Clear Channel 6 global interrupt flag, transfer complete flag and transfer error flag
	DMA1->IFCR = DMA_IFCR_CGIF6;
	DMA1->IFCR = DMA_IFCR_CTCIF6;
	DMA1->IFCR = DMA_IFCR_CTEIF6;
	// Clear Channel 7 global interrupt flag, transfer complete flag and transfer error flag
    DMA1->IFCR = DMA_IFCR_CGIF7;
	DMA1->IFCR = DMA_IFCR_CTCIF7;
	DMA1->IFCR = DMA_IFCR_CTEIF7;


}


/* Инициализация USART2 - все настройки стандартные: асинхронный режим, 
   скорость 115200 бит/с, длина слова 8 бит, 1 стоп бит, контроля чётности нет,
   есть приём и передача
 */
void USART2_init()
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
	GPIOA->CRL |= GPIO_CRL_CNF2_1; // PA2 (TX): output 50MHz - Alt-function Push-pull
	
	GPIOA->CRL |= GPIO_CRL_CNF3_0; // PA3 (RX): input floating

	DMA1_init_for_USART2();

	/* настраиваем регистры USART2: асинхронный режим, включаем приём и передачу,
	устанавливаем скорость передачи - baudrate (пока что фиксированная
	скорость - 115200 бит/с), длина слова - 8 бит, 1 стоп бит */
	if ((USART2->CR1 & USART_CR1_UE) != USART_CR1_UE) // проверяем, вдруг включен USART
	{
		USART2->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
		USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
	}
	USART2->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
	USART2->CR3 &= ~(USART_CR3_SCEN | USART_CR3_IREN | USART_CR3_HDSEL);
	USART2->BRR = 0x138; // рассчитывалось по даташиту, равно 19.53
	
	//Enable DMA Mode for reception (DMAR) and transmission (DMAT)
	USART2->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
	
    USART2->DR = 0x00;

	// USART2->SR &= ~USART_SR_TC;

	// NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 1));
    // NVIC_EnableIRQ(USART2_IRQn);
	// Enable USART2 and DMA Channel 6 for USART RX
	USART2->CR1 |= USART_CR1_UE; // enable USART2
	DMA1_Channel6->CCR |= DMA_CCR_EN;
	// DMA1_Channel7->CCR |= DMA_CCR_EN;


}

void process_usart_data(uint8_t* data, size_t length)
{
	const uint8_t* local_data = data;
}

// Details here: https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx
void check_rx_uart()
{
	static size_t old_pos;
	size_t pos;
	
	pos = ARRAY_LEN(usart_rx_dma_buffer) - DMA_CNDTR_NDT;
	if (pos != old_pos)
	{
		if (pos > old_pos)
		{
			process_usart_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);
		}
		else
		{
			process_usart_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);
			if (pos > 0)
			{
				process_usart_data(&usart_rx_dma_buffer[0], pos);
			}
		}
	}
	old_pos = pos;
	if (old_pos == ARRAY_LEN(usart_rx_dma_buffer))
	{
		old_pos = 0;
	}
}


void serial_write(uint16_t size_of_data)
{
	if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
	{
		DMA1_Channel7->CCR &= ~DMA_CCR_EN;
	    DMA1_Channel7->CNDTR &= ~DMA_CNDTR_NDT;
	    DMA1_Channel7->CNDTR = size_of_data;
        DMA1_Channel7->CCR |= DMA_CCR_EN;
	
	}
	// while (!(USART2->SR & USART_SR_TC)) {}
	// while (!flag_tx) {}
	// flag_tx = 0;
}

void USART_TX (uint8_t* dt, uint16_t sz)
{
  uint16_t ind = 0;
  while (ind<sz)
  {
    while (READ_BIT(USART2->SR, USART_SR_TXE) != (USART_SR_TXE)) {}
    USART2->DR = (uint16_t)dt[ind];
    ind++;
  }
}




int main() {

	system_config();
	sys_tick_init();
	GPIO_setup();

    GREEN_LED_OFF;

	USART2_init();

	char sample_string[40];
	int i = 0;
	// strncpy(usart_tx_dma_buffer, "Hi there! Something doesnt work \n", ARRAY_LEN(usart_tx_dma_buffer));
	while(1) 
	{
		delay_ms(10);
		serial_write(strlen(usart_tx_dma_buffer));
		// if(i>1023) i=0;
        // sprintf(str1,"String %04d\r\n",i);
        // USART_TX((uint8_t*)str1,13);
        // i++;
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
    	// SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
    	if(SysTick_CNT > 0)  SysTick_CNT--;
    }
}

// Обработчик прерывания от периферии (USART2-RX) к памяти
extern "C" 
{
	void DMA1_Channel6_IRQHandler(void)
	{
		if ((DMA1->ISR & DMA_ISR_TCIF6) == DMA_ISR_TCIF6) // сработало прерывание по окончанию передачи
		{
			DMA1->IFCR = DMA_IFCR_CTCIF6; // Clear TCIF6 in DMA_ISR register
		}
		else if ((DMA1->ISR & DMA_ISR_TEIF6) == DMA_ISR_TEIF6) // произошла ошибка передачи
		{
			// выключаем оба канала DMA
			DMA1_Channel6->CCR &= ~DMA_CCR_EN;
			DMA1_Channel7->CCR &= ~DMA_CCR_EN;
		}
	}
}

// обработчик прерывания от памяти к периферии (USART2-TX)
extern "C" 
{
	void DMA1_Channel7_IRQHandler(void)
	{
		GREEN_LED_ON;
		GREEN_LED_OFF;
		if ((DMA1->ISR & DMA_ISR_TCIF7) == DMA_ISR_TCIF7) // сработало прерывание по окончанию передачи
		{
			DMA1->IFCR = DMA_IFCR_CTCIF7;
			if ((USART2->SR & USART_SR_TC) == USART_SR_TC)
			{
			    flag_tx = 1;
			}
			// serial_write(strlen(usart_tx_dma_buffer));
		}
		else if ((DMA1->ISR & DMA_ISR_TEIF7) == DMA_ISR_TEIF7)
		{
			// выключаем оба канала DMA
			DMA1_Channel6->CCR &= ~DMA_CCR_EN;
			DMA1_Channel7->CCR &= ~DMA_CCR_EN;
		}
		GREEN_LED_ON;
		GREEN_LED_OFF;
	}
}

extern "C"
{
	void USART2_IRQHandler(void) 
	{
    /* Implement other events when needed */
	}
}