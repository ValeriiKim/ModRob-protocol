#ifndef TIMER2_H
#define TIMER2_H

#include "stm32f1xx.h"

/** Этот таймер нужен только для отсчёта микросекунд с начала запуска 
 * таймера, что необходимо для работы протокола (ноды модуля) 
  */
namespace timer2
{
    uint64_t micros = 0;
    uint16_t previous_counter = 0; 
    uint32_t overflow_count = 0;

    void tim2_setup()
    {
    /** (1) Включаем тактирование таймера 2
     *  (2) Настраиваем режим Upcounting: CMS = 00, DIR = 0
     *  (3) Включаем буферизацию регистра TIM2_ARR (preload feature)
     *  (4) Only counter overflow/underflow generates an update interrupt 
     *  (5) UEV enabled - включаем генерацию update event
     *  (6) Выбираем величину прескейлера, можно взять его равным 71, тогда 
     *      частота таймера будет равна f_clk_CNT = 72 МГц / (71 + 1) = 1 МГц, т.е. 
     *      один период сигнала таймера будет равен 1 мкс, т.е. одному тику таймера
     *  (7) Выбираем максимальную величину регистра auto-reload
     *  (8) Установка приоритета прерываний
     *  (9) Включение прерываний
     *  (10) Включение прерываний по переполнению счётчика
     *  Важно! Частота шины APB1 равна 36 МГц, однако для таймера 2 она удваивается - 72 МГц
    */
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;          // (1)
        TIM2->CR1 &= ~(TIM_CR1_CMS | TIM_CR1_DIR);   // (2)
        TIM2->CR1 |= TIM_CR1_ARPE;                   // (3)
        TIM2->CR1 |= TIM_CR1_URS;                    // (4)
        TIM2->CR1 &= ~TIM_CR1_UDIS;                  // (5)

        TIM2->PSC = 71;                              // (6)
        TIM2->ARR = 65535;                           // (7)

        NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 1)); // (8)
		NVIC_EnableIRQ(TIM2_IRQn);                   // (9)
        TIM2->DIER |= TIM_DIER_UIE;                  // (10)
    }

    void tim2_start()
    {
        TIM2->CR1 |= TIM_CR1_CEN;
    }

    inline uint64_t get_micros()
    {
        // Используем число переполнений, умноженных на число в регистре ARR + текущий счётчик
        return (overflow_count * 65535 + TIM2->CNT);
    }

extern "C"
    {
        /* С каждым прерыванием увеличиваем счётчик переполнений */
        void TIM2_IRQHandler()
        {
            if (TIM2->SR & TIM_SR_UIF)
            {
                TIM2->SR &= ~TIM_SR_UIF;
                overflow_count++;
            }
        }
    }
}


#endif