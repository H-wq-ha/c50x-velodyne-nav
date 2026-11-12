#include "board.h"
/* FreeRTOS port.c provides SVC, PendSV and SysTick through FreeRTOSConfig.h. */
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
void DMA1_Channel4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void HardFault_Handler(void) { Board_Fatal(0x10U); }
void MemManage_Handler(void) { Board_Fatal(0x11U); }
void BusFault_Handler(void) { Board_Fatal(0x12U); }
void UsageFault_Handler(void) { Board_Fatal(0x13U); }
