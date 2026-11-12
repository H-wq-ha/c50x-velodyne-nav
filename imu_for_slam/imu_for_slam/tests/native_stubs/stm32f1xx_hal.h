#ifndef NATIVE_TEST_STM32_HAL_H
#define NATIVE_TEST_STM32_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct { uint32_t unused; } UART_HandleTypeDef;
typedef struct { uint32_t unused; } DMA_HandleTypeDef;
typedef struct { uint32_t unused; } TIM_HandleTypeDef;
void HAL_Delay(uint32_t milliseconds);
#endif
