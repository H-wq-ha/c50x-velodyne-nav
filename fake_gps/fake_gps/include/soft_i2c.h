#ifndef SOFT_I2C_H
#define SOFT_I2C_H
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
    uint32_t half_period_us;
} soft_i2c_t;

/* 教程式基本操作：高电平表示释放开漏输出；ACK=0，NACK=1。 */
void SoftI2C_Init(soft_i2c_t *bus);
HAL_StatusTypeDef SoftI2C_Start(soft_i2c_t *bus);
HAL_StatusTypeDef SoftI2C_Stop(soft_i2c_t *bus);
HAL_StatusTypeDef SoftI2C_SendByte(soft_i2c_t *bus, uint8_t byte);
HAL_StatusTypeDef SoftI2C_ReceiveByte(soft_i2c_t *bus, uint8_t *byte);
HAL_StatusTypeDef SoftI2C_SendAck(soft_i2c_t *bus, uint8_t ack_bit);
HAL_StatusTypeDef SoftI2C_ReceiveAck(soft_i2c_t *bus, uint8_t *ack_bit);

/* Device addresses here are 7-bit; address+R/W is formed inside the driver. */
HAL_StatusTypeDef SoftI2C_Probe(soft_i2c_t *bus, uint8_t address);
HAL_StatusTypeDef SoftI2C_Write(soft_i2c_t *bus, uint8_t address, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef SoftI2C_WriteReg(soft_i2c_t *bus, uint8_t address, uint8_t reg,
                                 const uint8_t *data, uint16_t size);
HAL_StatusTypeDef SoftI2C_ReadReg(soft_i2c_t *bus, uint8_t address, uint8_t reg,
                                uint8_t *data, uint16_t size);
HAL_StatusTypeDef SoftI2C_Recover(soft_i2c_t *bus);
#endif
