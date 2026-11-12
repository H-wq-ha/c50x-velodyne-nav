#ifndef APP_RTOS_H
#define APP_RTOS_H
#include <stdint.h>

typedef struct {
    uint32_t imu_reads;
    uint32_t serial_samples;
    uint32_t oled_frames;
    uint32_t imu_overruns;
    uint32_t oled_overruns;
    uint32_t imu_read_max_us;
    uint32_t oled_frame_max_us;
    uint32_t sample_queue_drops;
    uint32_t log_queue_drops;
    uint32_t imu_online;
    uint32_t oled_online;
    uint32_t imu_hz_x10;
    uint32_t serial_hz_x10;
    uint32_t oled_hz_x10;
} app_metrics_t;
extern volatile app_metrics_t g_app_metrics;
#endif
