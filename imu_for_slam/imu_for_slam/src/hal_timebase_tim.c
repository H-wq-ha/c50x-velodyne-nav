#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

TIM_HandleTypeDef htim2;

/* HAL_Init and HAL_RCC_ClockConfig both call this override. TIM2 is the HAL
   1ms tick and microsecond phase; SysTick belongs exclusively to FreeRTOS. */
HAL_StatusTypeDef HAL_InitTick(uint32_t priority)
{
    if (priority >= (1U << __NVIC_PRIO_BITS)) return HAL_ERROR;
    __HAL_RCC_TIM2_CLK_ENABLE();
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U) timer_clock *= 2U;
    if (timer_clock < 1000000U || timer_clock % 1000000U != 0U) return HAL_ERROR;
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    if (htim2.Instance == TIM2) {
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE);
        __HAL_TIM_DISABLE(&htim2);
    }
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = timer_clock/1000000U-1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) return HAL_ERROR;
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_NVIC_SetPriority(TIM2_IRQn, priority, 0U);
    uwTickPrio = priority;
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    return HAL_TIM_Base_Start_IT(&htim2);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM2) HAL_IncTick();
}
void HAL_SuspendTick(void) { __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_UPDATE); }
void HAL_ResumeTick(void) { __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE); }

void HAL_Delay(uint32_t ms)
{
    if (ms == 0U) return;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        configASSERT(__get_IPSR() == 0U);
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        uint32_t start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick()-start) < ms) __NOP();
    }
}
