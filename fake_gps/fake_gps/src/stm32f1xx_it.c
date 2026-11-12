#include "board.h"

/* FreeRTOS supplies SVC, PendSV and SysTick. HAL uses TIM4. */
void DMA1_Channel4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void DMA1_Channel7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_tx); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void NMI_Handler(void)
{
    if (__HAL_RCC_GET_IT(RCC_IT_CSS) != RESET) {
        __HAL_RCC_CLEAR_IT(RCC_IT_CSS);
        Board_Fatal(0x20U); /* Runtime HSE loss: output timing is invalid. */
    }
    Board_Fatal(0x21U);
}
void HardFault_Handler(void) { Board_Fatal(0x10U); }
void MemManage_Handler(void) { Board_Fatal(0x11U); }
void BusFault_Handler(void) { Board_Fatal(0x12U); }
void UsageFault_Handler(void) { Board_Fatal(0x13U); }
