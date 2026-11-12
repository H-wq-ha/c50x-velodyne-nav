#ifndef BOARD_H
#define BOARD_H
#include "app_config.h"
#include "soft_i2c.h"
#include "board_time.h"
#include <stdbool.h>
typedef enum {
    APP_STAGE_RESET = 0, APP_STAGE_CLOCK, APP_STAGE_PERIPHERALS,
    APP_STAGE_INITIALIZING, APP_STAGE_RUNNING, APP_STAGE_IMU_OFFLINE, APP_STAGE_FATAL
} app_stage_t;
typedef struct {
    uint32_t sysclk_hz;
    bool hse_in_use;
} board_clock_t;
#if APP_ENABLE_OLED
extern soft_i2c_t oled_i2c;
#endif
extern soft_i2c_t imu_i2c;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern TIM_HandleTypeDef htim2;
extern volatile app_stage_t g_app_stage;
extern volatile uint32_t g_fault_code;
extern board_clock_t g_board_clock;
void SystemClock_Config(void);
bool Board_PeripheralsInit(void);
void Board_SetLed(bool on);
void Board_Fatal(uint32_t code);
#endif
