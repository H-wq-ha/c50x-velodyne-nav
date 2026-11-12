#ifndef APP_RTOS_H
#define APP_RTOS_H
#include "clock_sync.h"
#include "imu_units.h"
typedef struct { uint64_t monotonic_us; uint32_t sequence; } imu_event_t;
typedef struct {
    volatile uint32_t imu_online, who_am_i, drdy_count, imu_reads, samples_sent;
    volatile uint32_t lost_samples, imu_io_errors, sample_queue_drops;
    volatile uint32_t capture_overruns, capture_queue_drops, read_races;
    volatile uint32_t read_max_us, read_rate_millihz, send_rate_millihz;
    volatile uint32_t imu_stack_free, gps_stack_free, uart_stack_free;
    volatile uint32_t drdy_timeouts, invalid_time_drops;
} app_metrics_t;
extern volatile app_metrics_t g_app;
void App_Start(void) __attribute__((noreturn));
void App_ImuCaptureFromISR(uint64_t captured_us, bool overcapture);
#endif
