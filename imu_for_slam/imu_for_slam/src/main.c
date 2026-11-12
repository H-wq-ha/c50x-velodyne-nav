#include "board.h"
#include "mpu6050.h"
#if APP_ENABLE_OLED
#include "oled_ssd1306.h"
#endif
#include "serial_hal.h"
#include "app_rtos.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_CAPACITY 160U
#define IMU_STACK_WORDS 640U
#define UART_STACK_WORDS 768U
#if APP_ENABLE_OLED
#define OLED_STACK_WORDS 768U
#endif

typedef struct { char text[LOG_CAPACITY]; } app_log_t;
#if APP_ENABLE_OLED
typedef struct {
    imu_sample_t sample;
    bool online;
    bool have_sample;
    uint8_t who_am_i;
} imu_snapshot_t;
#endif

static mpu6050_t imu;
#if APP_ENABLE_OLED
static oled_t oled;
static imu_snapshot_t shared_imu;
#endif
volatile app_metrics_t g_app_metrics;
static StaticQueue_t sample_queue_control, log_queue_control;
static uint8_t sample_queue_storage[APP_SAMPLE_QUEUE_LENGTH*sizeof(imu_sample_t)];
static uint8_t log_queue_storage[APP_LOG_QUEUE_LENGTH*sizeof(app_log_t)];
static QueueHandle_t sample_queue, log_queue;
static StaticTask_t imu_task_control, uart_task_control;
static StackType_t imu_stack[IMU_STACK_WORDS];
static StackType_t uart_stack[UART_STACK_WORDS];
static TaskHandle_t imu_task_handle, uart_task_handle;
#if APP_ENABLE_OLED
static StaticTask_t oled_task_control;
static StackType_t oled_stack[OLED_STACK_WORDS];
static TaskHandle_t oled_task_handle;
#endif

/* IMU/OLED producers queue logs. Only uart_task writes the UART ring once
   scheduling starts, so its HAL state is never touched by concurrent tasks. */
static void app_log(const char *format, ...)
{
    app_log_t message = {0};
    va_list args;
    va_start(args, format);
    int length = vsnprintf(message.text, sizeof(message.text), format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= sizeof(message.text) ||
        xQueueSend(log_queue, &message, 0U) != pdPASS) {
        taskENTER_CRITICAL();
        ++g_app_metrics.log_queue_drops;
        taskEXIT_CRITICAL();
    }
}
static void report_imu(void)
{
    for (unsigned i = 0; i < 2U; ++i) {
        if (!imu.probes[i].attempted) continue;
        app_log("@STATUS,IMU_PROBE,ADDR=0x%02X,WHO=0x%02X,BUS=%u\r\n",
                (unsigned)(0x68U+i), imu.probes[i].who_am_i,
                (unsigned)imu.probes[i].status);
    }
    app_log("@STATUS,IMU_%s,ADDR=0x%02X,WHO=0x%02X,BUS=%u,MODEL=%s,STAGE=%s,REG=0x%02X\r\n",
            g_app_metrics.imu_online ? "OK" : "OFFLINE", imu.address, imu.who_am_i,
            (unsigned)imu.last_status, MPU6050_ModelName(&imu), imu.stage, imu.last_register);
}
static void set_imu_online(bool online)
{
#if APP_ENABLE_OLED
    taskENTER_CRITICAL();
    shared_imu.online = online;
    shared_imu.have_sample = false;
    shared_imu.who_am_i = imu.who_am_i;
    taskEXIT_CRITICAL();
#endif
    g_app_metrics.imu_online = online;
    g_app_stage = online ? APP_STAGE_RUNNING : APP_STAGE_IMU_OFFLINE;
}
#if APP_ENABLE_OLED
static imu_snapshot_t get_snapshot(void)
{
    imu_snapshot_t snapshot;
    taskENTER_CRITICAL();
    snapshot = shared_imu;
    taskEXIT_CRITICAL();
    return snapshot;
}
#endif
static void wait_period(TickType_t *wake, TickType_t period, volatile uint32_t *overruns)
{
    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now-*wake) >= period) {
        ++*overruns;
        *wake = now; /* Do not send back-to-back catch-up samples after a missed deadline. */
    }
    vTaskDelayUntil(wake, period);
}
static void imu_task(void *argument)
{
    uint32_t sequence = 0U;
    const TickType_t period = pdMS_TO_TICKS(1000U/APP_IMU_RATE_HZ);
    for (;;) {
        SoftI2C_Recover(&imu_i2c);
        bool online = MPU6050_Init(&imu, &imu_i2c);
        set_imu_online(online);
        report_imu();
        if (!online) {
            vTaskDelay(pdMS_TO_TICKS(APP_RETRY_PERIOD_MS));
            continue;
        }
        TickType_t wake = xTaskGetTickCount();
        while (online) {
            imu_sample_t sample;
            uint32_t started = Board_Micros();
            if (MPU6050_Read(&imu, &sample) != IMU_READ_OK) {
                online = false;
                set_imu_online(false);
                report_imu();
                break;
            }
            uint32_t elapsed = Board_Micros()-started;
            if (elapsed > g_app_metrics.imu_read_max_us) g_app_metrics.imu_read_max_us = elapsed;
            sample.sequence = sequence++;
            ++g_app_metrics.imu_reads;
#if APP_ENABLE_OLED
            taskENTER_CRITICAL();
            shared_imu.sample = sample;
            shared_imu.have_sample = true;
            taskEXIT_CRITICAL();
#endif
            if (xQueueSend(sample_queue, &sample, 0U) != pdPASS) ++g_app_metrics.sample_queue_drops;
            wait_period(&wake, period, &g_app_metrics.imu_overruns);
        }
        vTaskDelay(pdMS_TO_TICKS(APP_RETRY_PERIOD_MS));
    }
}
#if APP_ENABLE_OLED /* 暂停显示绘制、设备初始化、重试和整帧刷新。 */
static bool draw_display(const imu_snapshot_t *snapshot)
{
    if (!OLED_BeginFrame(&oled)) return false;
    char text[32];
    const char *model = snapshot->who_am_i == 0x70U ? "MPU6500" : "MPU6050";
    if (!snapshot->online) {
        OLED_DrawLine(&oled, 0, "IMU OFFLINE");
        OLED_DrawLine(&oled, 1, "CHECK PB10 / PB11");
        OLED_DrawLine(&oled, 2, "MPU60XX 0x68 / 0x69");
        OLED_DrawLine(&oled, 3, "POWER 3.3V / GND");
        OLED_DrawLine(&oled, 4, "RETRY EVERY 1s");
    } else if (!snapshot->have_sample) {
        OLED_DrawLine(&oled, 0, model);
        OLED_DrawLine(&oled, 2, "WAIT FIRST READ");
    } else {
        snprintf(text, sizeof(text), "%s SI %luHz", model, (unsigned long)APP_IMU_RATE_HZ);
        OLED_DrawLine(&oled, 0, text);
        for (unsigned axis = 0; axis < 3U; ++axis) {
            char value[16];
            IMU_FormatFixed(value, sizeof(value), snapshot->sample.accel_mps2[axis], 3U);
            snprintf(text, sizeof(text), "A%c %s m/s2", "XYZ"[axis], value);
            OLED_DrawLine(&oled, (uint8_t)(1U+axis), text);
            IMU_FormatFixed(value, sizeof(value), snapshot->sample.gyro_rads[axis], 4U);
            snprintf(text, sizeof(text), "G%c %s rad/s", "XYZ"[axis], value);
            OLED_DrawLine(&oled, (uint8_t)(4U+axis), text);
        }
    }
    char temperature[16];
    if (snapshot->have_sample) {
        IMU_FormatFixed(temperature, sizeof(temperature), snapshot->sample.temperature_c, 1U);
        snprintf(text, sizeof(text), "T%sC %s%luMHz", temperature,
                 g_board_clock.hse_in_use ? "HSE" : "HSI",
                 (unsigned long)(g_board_clock.sysclk_hz/1000000U));
    } else {
        snprintf(text, sizeof(text), "SYS %s %luMHz", g_board_clock.hse_in_use ? "HSE" : "HSI",
                 (unsigned long)(g_board_clock.sysclk_hz/1000000U));
    }
    OLED_DrawLine(&oled, 7, text);
    OLED_QueueRefresh(&oled);
    return true;
}
static void oled_task(void *argument)
{
    const TickType_t period = pdMS_TO_TICKS(APP_OLED_PERIOD_MS);
    for (;;) {
        SoftI2C_Recover(&oled_i2c);
        bool online = OLED_Init(&oled, &oled_i2c);
        g_app_metrics.oled_online = online;
        app_log(online ? "@STATUS,OLED_OK\r\n" : "@STATUS,OLED_OFFLINE\r\n");
        if (!online) {
            vTaskDelay(pdMS_TO_TICKS(5000U));
            continue;
        }
        TickType_t wake = xTaskGetTickCount();
        while (online) {
            imu_snapshot_t snapshot = get_snapshot();
            uint32_t started = Board_Micros();
            online = draw_display(&snapshot);
            while (online && oled.pending_pages != 0U) {
                /* Separate bus and lower priority: IMU/UART preempt between any
                   GPIO operations; the current display frame remains immutable. */
                online = OLED_Service(&oled) == HAL_OK;
            }
            if (!online) {
                g_app_metrics.oled_online = 0U;
                app_log("@STATUS,OLED_IO_ERROR\r\n");
                break;
            }
            ++g_app_metrics.oled_frames;
            uint32_t elapsed = Board_Micros()-started;
            if (elapsed > g_app_metrics.oled_frame_max_us) g_app_metrics.oled_frame_max_us = elapsed;
            wait_period(&wake, period, &g_app_metrics.oled_overruns);
        }
        vTaskDelay(pdMS_TO_TICKS(5000U));
    }
}
#endif /* APP_ENABLE_OLED */
static void update_rates(void)
{
    static uint32_t previous_us, previous_imu, previous_serial, previous_oled;
    uint32_t now = Board_Micros();
    if (previous_us == 0U) {
        previous_us = now;
        previous_imu = g_app_metrics.imu_reads;
        previous_serial = g_app_metrics.serial_samples;
        previous_oled = g_app_metrics.oled_frames;
        return;
    }
    uint32_t interval = now-previous_us;
    if (interval < 1000000U) return;
    taskENTER_CRITICAL();
    uint32_t imu_count = g_app_metrics.imu_reads;
    uint32_t serial_count = g_app_metrics.serial_samples;
    uint32_t oled_count = g_app_metrics.oled_frames;
    taskEXIT_CRITICAL();
    g_app_metrics.imu_hz_x10 = (uint32_t)((uint64_t)(imu_count-previous_imu)*10000000ULL/interval);
    g_app_metrics.serial_hz_x10 = (uint32_t)((uint64_t)(serial_count-previous_serial)*10000000ULL/interval);
    g_app_metrics.oled_hz_x10 = (uint32_t)((uint64_t)(oled_count-previous_oled)*10000000ULL/interval);
    previous_us = now;
    previous_imu = imu_count;
    previous_serial = serial_count;
    previous_oled = oled_count;
}
static void report_stats(void)
{
    char line[LOG_CAPACITY];
    uint32_t a = g_app_metrics.imu_hz_x10;
    uint32_t b = g_app_metrics.serial_hz_x10;
    uint32_t c = g_app_metrics.oled_hz_x10;
    snprintf(line, sizeof(line),
             "@STATUS,RATE,IMU=%lu.%lu,SERIAL=%lu.%lu,OLED=%lu.%lu,IMU_OVR=%lu,OLED_OVR=%lu,QDROP=%lu\r\n",
             (unsigned long)(a/10U), (unsigned long)(a%10U),
             (unsigned long)(b/10U), (unsigned long)(b%10U),
             (unsigned long)(c/10U), (unsigned long)(c%10U),
             (unsigned long)g_app_metrics.imu_overruns, (unsigned long)g_app_metrics.oled_overruns,
             (unsigned long)g_app_metrics.sample_queue_drops);
    Serial_Log(line);
    snprintf(line, sizeof(line),
             "@STATUS,RTOS,READ_MAX_US=%lu,FRAME_MAX_US=%lu,TXDROP=%lu,RXDROP=%lu,UARTERR=%lu,LOGDROP=%lu\r\n",
             (unsigned long)g_app_metrics.imu_read_max_us, (unsigned long)g_app_metrics.oled_frame_max_us,
             (unsigned long)g_serial_stats.tx_dropped, (unsigned long)g_serial_stats.rx_dropped,
             (unsigned long)g_serial_stats.uart_errors, (unsigned long)g_app_metrics.log_queue_drops);
    Serial_Log(line);
#if APP_ENABLE_OLED
    snprintf(line, sizeof(line), "@STATUS,STACK_FREE_WORDS,IMU=%lu,UART=%lu,OLED=%lu\r\n",
             (unsigned long)uxTaskGetStackHighWaterMark(imu_task_handle),
             (unsigned long)uxTaskGetStackHighWaterMark(uart_task_handle),
             (unsigned long)uxTaskGetStackHighWaterMark(oled_task_handle));
#else
    snprintf(line, sizeof(line), "@STATUS,STACK_FREE_WORDS,IMU=%lu,UART=%lu,OLED=DISABLED\r\n",
             (unsigned long)uxTaskGetStackHighWaterMark(imu_task_handle),
             (unsigned long)uxTaskGetStackHighWaterMark(uart_task_handle));
#endif
    Serial_Log(line);
}
static void uart_task(void *argument)
{
    bool streaming = true;
    bool led = false;
    uint32_t last_led = HAL_GetTick();
    serial_mode_t mode = APP_BINARY_DEFAULT ? SERIAL_BINARY : SERIAL_TEXT;
    for (;;) {
        Serial_Service();
        update_rates();
        uint32_t led_period = g_app_metrics.imu_online ? 500U : 125U;
        if ((uint32_t)(HAL_GetTick()-last_led) >= led_period) {
            led = !led;
            Board_SetLed(led);
            last_led = HAL_GetTick();
        }
        serial_command_t command;
        while ((command = Serial_PollCommand()) != SERIAL_CMD_NONE) {
            switch (command) {
            case SERIAL_CMD_PING: Serial_Log("@PONG\r\n"); break;
            case SERIAL_CMD_START: streaming = true; Serial_Log("@STATUS,START\r\n"); break;
            case SERIAL_CMD_STOP: streaming = false; Serial_Log("@STATUS,STOP\r\n"); break;
            case SERIAL_CMD_TEXT: mode = SERIAL_TEXT; Serial_Log("@STATUS,TEXT\r\n"); break;
            case SERIAL_CMD_HEX: mode = SERIAL_BINARY; Serial_Log("@STATUS,HEX\r\n"); break;
            case SERIAL_CMD_STATS: report_stats(); break;
            default: Serial_Log("@ERROR,UNKNOWN_COMMAND\r\n"); break;
            }
        }
        app_log_t message;
        for (unsigned i = 0; i < 2U && xQueueReceive(log_queue, &message, 0U) == pdPASS; ++i) {
            Serial_Log(message.text);
        }
        imu_sample_t sample;
        if (xQueueReceive(sample_queue, &sample, pdMS_TO_TICKS(1U)) == pdPASS && streaming) {
            if (Serial_SendSample(&sample, mode)) ++g_app_metrics.serial_samples;
        }
    }
}
void vApplicationGetIdleTaskMemory(StaticTask_t **control, StackType_t **stack, uint32_t *size)
{
    static StaticTask_t idle_control;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *control = &idle_control;
    *stack = idle_stack;
    *size = configMINIMAL_STACK_SIZE;
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name) { Board_Fatal(0x21U); }

int main(void)
{
    if (HAL_Init() != HAL_OK) Board_Fatal(3U);
    SystemClock_Config();
    if (!Board_PeripheralsInit()) Board_Fatal(4U);
    if (!Serial_Init()) Board_Fatal(5U);
    g_app_stage = APP_STAGE_INITIALIZING;
    sample_queue = xQueueCreateStatic(APP_SAMPLE_QUEUE_LENGTH, sizeof(imu_sample_t),
                                     sample_queue_storage, &sample_queue_control);
    log_queue = xQueueCreateStatic(APP_LOG_QUEUE_LENGTH, sizeof(app_log_t),
                                  log_queue_storage, &log_queue_control);
    if (sample_queue == NULL || log_queue == NULL) Board_Fatal(6U);
    vQueueAddToRegistry(sample_queue, "IMU_SAMPLES");
    vQueueAddToRegistry(log_queue, "APP_LOGS");
    char boot[LOG_CAPACITY];
    snprintf(boot, sizeof(boot),
             "@STATUS,BOOT,%s,%luHz,%luHz_IMU,%luHz_OLED,%lu_BAUD,FreeRTOS,HEX\r\n",
             g_board_clock.hse_in_use ? "HSE" : "HSI_FALLBACK",
             (unsigned long)g_board_clock.sysclk_hz, (unsigned long)APP_IMU_RATE_HZ,
             (unsigned long)(APP_ENABLE_OLED ? 1000U/APP_OLED_PERIOD_MS : 0U),
             (unsigned long)APP_UART_BAUD);
    Serial_Log(boot);
    imu_task_handle = xTaskCreateStatic(imu_task, "IMU_200Hz", IMU_STACK_WORDS, NULL, 4U,
                                       imu_stack, &imu_task_control);
    uart_task_handle = xTaskCreateStatic(uart_task, "UART_200Hz", UART_STACK_WORDS, NULL, 3U,
                                        uart_stack, &uart_task_control);
#if APP_ENABLE_OLED
    oled_task_handle = xTaskCreateStatic(oled_task, "OLED_10Hz", OLED_STACK_WORDS, NULL, 1U,
                                        oled_stack, &oled_task_control);
    if (oled_task_handle == NULL) Board_Fatal(7U);
#else
    Serial_Log("@STATUS,OLED_DISABLED\r\n");
#endif
    if (imu_task_handle == NULL || uart_task_handle == NULL) Board_Fatal(7U);
    vTaskStartScheduler();
    Board_Fatal(8U); /* Scheduler returning is a fault, never an empty default task. */
}
