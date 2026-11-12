#include "app_rtos.h"
#include "board.h"
#include "app_config.h"
#include "mpu6050.h"
#include "serial_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

volatile app_metrics_t g_app;
static QueueHandle_t capture_queue, sample_queue;
static StaticQueue_t capture_control, sample_control;
static uint8_t capture_storage[APP_CAPTURE_QUEUE_LENGTH * sizeof(imu_event_t)];
static uint8_t sample_storage[APP_SAMPLE_QUEUE_LENGTH * sizeof(imu_sample_t)];
static StaticTask_t gps_control, imu_control, uart_control;
static StackType_t gps_stack[384], imu_stack[512], uart_stack[512];
static TaskHandle_t gps_handle, imu_handle, uart_handle;
static mpu6050_t imu;

void App_ImuCaptureFromISR(uint64_t captured_us, bool overcapture)
{
    imu_event_t event = {captured_us, ++g_app.drdy_count};
    if (overcapture) {
        ++g_app.capture_overruns;
        ++g_app.lost_samples;
    }
    BaseType_t wake = pdFALSE;
    if (capture_queue != NULL && xQueueSendFromISR(capture_queue, &event, &wake) != pdPASS) {
        ++g_app.capture_queue_drops;
        ++g_app.lost_samples;
    }
    portYIELD_FROM_ISR(wake);
}

static void gps_task(void *argument)
{
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        GPS_Service();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1U));
    }
}

static void lost_sample(void)
{
    taskENTER_CRITICAL();
    ++g_app.lost_samples;
    taskEXIT_CRITICAL();
}

static void imu_task(void *argument)
{
    for (;;) {
        Board_EnableImuCapture(false);
        g_app.imu_online = 0U;
        xQueueReset(capture_queue);
        SoftI2C_Recover(&imu_i2c);
        if (!MPU6050_Init(&imu, &imu_i2c)) {
            g_app.who_am_i = imu.who_am_i;
            ++g_app.imu_io_errors;
            vTaskDelay(pdMS_TO_TICKS(APP_RETRY_PERIOD_MS));
            continue;
        }
        g_app.who_am_i = imu.who_am_i;
        /* Queue and input capture must be ready before enabling IMU INT. */
        xQueueReset(capture_queue);
        Board_EnableImuCapture(true);
        if (!MPU6050_EnableDataReady(&imu)) {
            ++g_app.imu_io_errors;
            Board_EnableImuCapture(false);
            vTaskDelay(pdMS_TO_TICKS(APP_RETRY_PERIOD_MS));
            continue;
        }
        g_app.imu_online = 1U;
        uint32_t last_sequence = 0U;
        for (;;) {
            imu_event_t event, newer;
            if (xQueueReceive(capture_queue, &event, pdMS_TO_TICKS(100U)) != pdPASS) {
                ++g_app.drdy_timeouts;
                break; /* Missing INT is visible; never fabricate 200Hz samples. */
            }
            /* No FIFO: only the latest register set remains readable. Discard
             * old timestamps, never read latest data repeatedly for backlog.
             */
            while (xQueueReceive(capture_queue, &newer, 0U) == pdPASS) {
                event = newer;
                lost_sample();
            }
            uint64_t started = Board_MonotonicUs();
            if (started < event.monotonic_us || started - event.monotonic_us >= 4000ULL) {
                lost_sample();
                continue;
            }
            sync_stamp_t stamp;
            if (!Board_Stamp(event.monotonic_us, &stamp)) {
                ++g_app.invalid_time_drops;
                continue; /* Wait for the first hardware PPS capture. */
            }
            imu_sample_t sample = {0};
            if (MPU6050_Read(&imu, &sample) != IMU_READ_OK) {
                ++g_app.imu_io_errors;
                lost_sample();
                break;
            }
            uint32_t elapsed = (uint32_t)(Board_MonotonicUs() - started);
            if (elapsed > g_app.read_max_us) g_app.read_max_us = elapsed;
            if (g_app.drdy_count != event.sequence) {
                ++g_app.read_races;
                lost_sample();
                continue; /* New sample arrived during read: association uncertain. */
            }
            sample.sequence = event.sequence;
            sample.monotonic_us = stamp.monotonic_us;
            sample.epoch_us = stamp.epoch_us;
            sample.hour_us = stamp.hour_us;
            sample.pps_count = stamp.pps_count;
            sample.flags = IMU_FLAG_TIME_VALID | IMU_FLAG_DRDY_CAPTURE | IMU_FLAG_ONLINE;
            if (g_gps.hse_in_use) sample.flags |= IMU_FLAG_HSE;
            if (imu.who_am_i == 0x70U) sample.flags |= IMU_FLAG_MPU6500;
            if (last_sequence && event.sequence != last_sequence + 1U) sample.flags |= IMU_FLAG_GAP;
            last_sequence = event.sequence;
            sample.lost_samples = g_app.lost_samples;
            ++g_app.imu_reads;
            if (xQueueSend(sample_queue, &sample, 0U) != pdPASS) {
                ++g_app.sample_queue_drops;
                lost_sample();
            }
        }
        Board_EnableImuCapture(false);
        g_app.imu_online = 0U;
        vTaskDelay(pdMS_TO_TICKS(APP_RETRY_PERIOD_MS));
    }
}

static void report_status(bool streaming)
{
    static uint64_t previous_us;
    static uint32_t previous_reads, previous_sent;
    uint64_t now = Board_MonotonicUs();
    if (previous_us && now > previous_us) {
        g_app.read_rate_millihz = (uint32_t)((uint64_t)(g_app.imu_reads - previous_reads)
                                            * 1000000000ULL / (now - previous_us));
        g_app.send_rate_millihz = (uint32_t)((uint64_t)(g_app.samples_sent - previous_sent)
                                            * 1000000000ULL / (now - previous_us));
    }
    previous_us = now;
    previous_reads = g_app.imu_reads;
    previous_sent = g_app.samples_sent;
    g_app.imu_stack_free = (uint32_t)uxTaskGetStackHighWaterMark(imu_handle);
    g_app.gps_stack_free = (uint32_t)uxTaskGetStackHighWaterMark(gps_handle);
    g_app.uart_stack_free = (uint32_t)uxTaskGetStackHighWaterMark(uart_handle);
    sync_stamp_t stamp;
    uint32_t flags = Board_Stamp(now, &stamp) ? IMU_FLAG_TIME_VALID : 0U;
    if (g_gps.hse_in_use) flags |= IMU_FLAG_HSE;
    if (g_app.imu_online) flags |= IMU_FLAG_ONLINE;
    if (streaming) flags |= IMU_FLAG_STREAMING;
    if (g_app.who_am_i == 0x70U) flags |= IMU_FLAG_MPU6500;
    uint32_t words[16] = {
        (uint32_t)(now / 1000ULL), g_gps.sysclk_hz, flags, g_app.who_am_i,
        g_app.drdy_count, g_app.imu_reads, g_app.samples_sent, g_app.lost_samples,
        g_app.imu_io_errors, g_app.sample_queue_drops, g_serial_stats.tx_dropped,
        g_app.read_max_us, g_app.read_rate_millihz, g_app.send_rate_millihz,
        g_gps.pps_count, g_gps.nmea_sent
    };
    Serial_SendStatus(words);
}

static void uart_task(void *argument)
{
    if (!Serial_Init()) Board_Fatal(0x35U);
    bool streaming = true;
    uint32_t last_status = HAL_GetTick(), last_led = last_status;
    for (;;) {
        Serial_Service();
        bool report = false;
        serial_command_t command;
        while ((command = Serial_PollCommand()) != SERIAL_CMD_NONE) {
            if (command == SERIAL_CMD_START) streaming = true;
            else if (command == SERIAL_CMD_STOP) streaming = false;
            report = true; /* PING/STATS/invalid all return a CRC-protected status. */
        }
        imu_sample_t sample;
        while (xQueueReceive(sample_queue, &sample, 0U) == pdPASS) {
            sample.flags |= IMU_FLAG_STREAMING;
            if (streaming && Serial_SendSample(&sample)) ++g_app.samples_sent;
        }
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_status) >= 1000U || report) {
            report_status(streaming);
            last_status = now;
        }
        uint32_t led_period = !g_app.imu_online ? 100U : (g_gps.hse_in_use ? 500U : 125U);
        if ((uint32_t)(now - last_led) >= led_period) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            last_led = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void App_Start(void)
{
    capture_queue = xQueueCreateStatic(APP_CAPTURE_QUEUE_LENGTH, sizeof(imu_event_t),
                                      capture_storage, &capture_control);
    sample_queue = xQueueCreateStatic(APP_SAMPLE_QUEUE_LENGTH, sizeof(imu_sample_t),
                                     sample_storage, &sample_control);
    if (!capture_queue || !sample_queue) Board_Fatal(0x36U);
    gps_handle = xTaskCreateStatic(gps_task, "GPS", 384U, NULL, 4U, gps_stack, &gps_control);
    imu_handle = xTaskCreateStatic(imu_task, "IMU_DRDY", 512U, NULL, 3U, imu_stack, &imu_control);
    uart_handle = xTaskCreateStatic(uart_task, "PC_UART", 512U, NULL, 2U, uart_stack, &uart_control);
    if (!gps_handle || !imu_handle || !uart_handle) Board_Fatal(0x37U);
    Board_TimerStart();
    vTaskStartScheduler();
    Board_Fatal(0x38U);
}
void vApplicationGetIdleTaskMemory(StaticTask_t **control, StackType_t **stack, uint32_t *size)
{
    static StaticTask_t idle_control;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *control = &idle_control;
    *stack = idle_stack;
    *size = configMINIMAL_STACK_SIZE;
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name) { Board_Fatal(0x39U); }
