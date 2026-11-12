#ifndef GPS_BOARD_H
#define GPS_BOARD_H
#include "stm32f1xx_hal.h"
#include "gps_time.h"
#include "board_time.h"
#include "clock_sync.h"
#include "soft_i2c.h"

typedef enum {
    GPS_STAGE_RESET, GPS_STAGE_HAL, GPS_STAGE_CLOCK,
    GPS_STAGE_PERIPHERALS, GPS_STAGE_RUNNING, GPS_STAGE_FATAL
} gps_stage_t;

typedef struct {
    volatile gps_stage_t stage;
    volatile uint32_t fault_code;
    volatile uint32_t sysclk_hz;
    volatile uint32_t timer_hz;
    volatile uint32_t hse_in_use;
    volatile uint32_t pps_count;
    volatile uint32_t nmea_sent;
    volatile uint32_t nmea_dropped;
    volatile uint32_t uart_errors;
    volatile uint32_t last_tx_tick;
} gps_diagnostics_t;

extern volatile gps_diagnostics_t g_gps;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern soft_i2c_t imu_i2c;
extern sync_clock_t g_sync_clock;
void SystemClock_Config(void);
void Board_PeripheralsInit(void);
void Board_TimerStart(void);
void Board_CaptureInit(void);
void Board_EnableImuCapture(bool enabled);
bool Board_Stamp(uint64_t monotonic_us, sync_stamp_t *stamp);
void GPS_Service(void);
void Board_Fatal(uint32_t code) __attribute__((noreturn));
bool GPS_TakeSentence(gps_time_t *time, uint32_t *sequence);
#endif
