#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    if (priority >= (1U << __NVIC_PRIO_BITS)) return HAL_ERROR;
    __HAL_RCC_TIM4_CLK_ENABLE();
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    uint32_t hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) hz *= 2U;
    if (hz < 1000000U || hz % 1000000U != 0U) return HAL_ERROR;
    TIM4->CR1 = 0U;
    TIM4->DIER = 0U;
    TIM4->PSC = hz / 1000000U - 1U;
    TIM4->ARR = 999U;
    TIM4->EGR = TIM_EGR_UG;
    TIM4->CNT = 0U;
    TIM4->SR = 0U;
    TIM4->DIER = TIM_DIER_UIE;
    uwTickPrio = priority;
    HAL_NVIC_SetPriority(TIM4_IRQn, priority, 0U);
    HAL_NVIC_ClearPendingIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    TIM4->CR1 = TIM_CR1_CEN;
    return HAL_OK;
}
void TIM4_IRQHandler(void)
{
    TIM4->SR = ~TIM_SR_UIF;
    HAL_IncTick();
}
void HAL_SuspendTick(void) { TIM4->DIER &= ~TIM_DIER_UIE; }
void HAL_ResumeTick(void) { TIM4->DIER |= TIM_DIER_UIE; }
void HAL_Delay(uint32_t ms)
{
    if (ms == 0U) return;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        configASSERT(__get_IPSR() == 0U);
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        uint32_t started = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - started) < ms) __NOP();
    }
}
