// Простой пример с мигающим светодиодом с использованием HAL для проверки работы микроконтроллера

// Библиотеки микроконтроллера
// #include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

// Библиотеки CanFestival
#include "canfestival.h"
#include "can_stm32.h"
#include "TestMaster.h"

volatile uint32_t tmpreg;
volatile uint32_t SysTick_CNT = 0;

#define SYSCLOCK 72000000U // максимальная частота контроллера

#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC


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

void Clock_Init()
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

/* Boot up the Phase Locked Loop to give us a 72MHz clock.. */
void SystemConfig(void)
{ 
	Clock_Init();
	// включение тактирования AFIO
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_AFIOEN);
	// сбрасываем настройки SWD и JTAG
	CLEAR_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG);
	// и устанавливаем режим 010 - JTAG-DP Disabled and SW-DP Enabled
	SET_BIT(AFIO->MAPR, AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

}

void SysTick_Init()
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

void GPIO_setup()
{
	// включаем тактирование портов А, В и С для отладки
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
	
	// ждём
	tmpreg = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
	
	// настраиваем порт С на выход push-pull с максимальной скоростью 50 МГц
	CLEAR_BIT(GPIOC->CRH, GPIO_CRH_CNF13);
	SET_BIT(GPIOC->CRH, GPIO_CRH_MODE13);
}

void delay_ms(uint32_t ms)
{
    CLEAR_BIT(SysTick->VAL, SysTick_VAL_CURRENT_Msk);
	SET_BIT(SysTick->VAL, SYSCLOCK/1000 - 1);
	SysTick_CNT = ms;
	while(SysTick_CNT) {}
}

// void LED_Init()
// {
//     LED_GPIO_CLK_ENABLE();
//     GPIO_InitTypeDef GPIO_InitStruct;
//     GPIO_InitStruct.Pin = LED_PIN;
//     GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//     GPIO_InitStruct.Pull = GPIO_NOPULL; //This was previously PULL_UP, which makes no sense on an output pin! No pullup resistors here.
//     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//     HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
// }

int main() {

	SystemConfig();
	SysTick_Init();
	GPIO_setup();
	SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
	// SysTick_Config(SYSCLOCK / 1000);

	while(1) {
		SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
		delay_ms(500);
		SET_BIT(GPIOC->BSRR, GPIO_BSRR_BS13);
		delay_ms(500);
	}

	return 0;
}

/* Нужно добавлять для обработчиков прерываний (пока непонятно для всех
или для только SysTick) обёртку, которая позволяет воспринимать 
участок кода как код языка С (НЕ С++). Без этого функции обработчиков
прерываний не хотят работать  */
extern "C" {
    void SysTick_Handler(void) 
    {
    	// SET_BIT(GPIOC->BSRR, GPIO_BSRR_BR13);
    	if(SysTick_CNT > 0)  SysTick_CNT--;
    }
}
