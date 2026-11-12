#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32f1xx_hal.h"
/* Tutorial-style GPIO software I2C; no I2C1/I2C2 alternate function. */
#define OLED_SCL_PORT GPIOB
#define OLED_SDA_PORT GPIOB
#define OLED_SCL_PIN GPIO_PIN_8
#define OLED_SDA_PIN GPIO_PIN_9
#define IMU_SCL_PORT GPIOB
#define IMU_SDA_PORT GPIOB
#define IMU_SCL_PIN GPIO_PIN_10
#define IMU_SDA_PIN GPIO_PIN_11
#define LED_PORT GPIOC
#define LED_PIN GPIO_PIN_13
#define APP_UART_BAUD 460800U
#define APP_IMU_RATE_HZ 200U
/* OLED代码暂时按条件编译注释关闭；改为1可恢复显示任务和GPIO初始化。 */
#ifndef APP_ENABLE_OLED
#define APP_ENABLE_OLED 0
#endif
#define APP_OLED_PERIOD_MS 100U
#define APP_RETRY_PERIOD_MS 1000U
#define APP_SOFT_I2C_HALF_PERIOD_US 2U /* SCL low minimum; retained field name. */
#define APP_SOFT_I2C_HIGH_PERIOD_US 1U
#define APP_SOFT_I2C_TIMEOUT_US 1000U
#define APP_OLED_CHUNK_BYTES 64U
#define APP_BINARY_DEFAULT 1
#define APP_SAMPLE_QUEUE_LENGTH 8U
#define APP_LOG_QUEUE_LENGTH 8U
#ifndef APP_FORCE_HSI
#define APP_FORCE_HSI 0
#endif
#if APP_IMU_RATE_HZ < 1 || APP_IMU_RATE_HZ > 1000 || (1000 % APP_IMU_RATE_HZ) != 0
#error "MPU6050 DLPF sample rate must divide 1000 Hz."
#endif
#if APP_OLED_CHUNK_BYTES < 1 || APP_OLED_CHUNK_BYTES > 128
#error "OLED chunk must contain 1..128 bytes."
#endif
#endif
