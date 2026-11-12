// HAL time base on TIM3: SysTick is reserved for FreeRTOS.

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();

    // 1 MHz counter, 1 kHz interrupt
    TIM3->CR1 = 0;
    TIM3->PSC = (uint16_t)((SystemCoreClock / 1000000U) - 1U);
    TIM3->ARR = 1000U - 1U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0;
    TIM3->DIER = TIM_DIER_UIE;
    TIM3->CR1 = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM3_IRQn, TickPriority, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    uwTickFreq = HAL_TICK_FREQ_DEFAULT;
    return HAL_OK;
}

void TIM3_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U) {
        TIM3->SR = (uint32_t)~TIM_SR_UIF;
        HAL_IncTick();
    }
}
