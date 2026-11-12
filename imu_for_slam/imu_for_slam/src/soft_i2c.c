#include "soft_i2c.h"
#include "board_time.h"
#include "app_config.h"
#include <stddef.h>

static void write_scl(soft_i2c_t *bus, GPIO_PinState level)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, level);
}
static void write_sda(soft_i2c_t *bus, GPIO_PinState level)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, level);
}
static GPIO_PinState read_sda(soft_i2c_t *bus)
{
    return HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin);
}
static void delay(soft_i2c_t *bus) { Board_DelayUs(bus->half_period_us); }
static void high_delay(soft_i2c_t *bus)
{
    /* Fast-mode low >=1.3us, high >=0.6us: use 2us / 1us minima.
       Legacy buses with a larger configured half-period retain that delay. */
    Board_DelayUs(bus->half_period_us == APP_SOFT_I2C_HALF_PERIOD_US ?
                  APP_SOFT_I2C_HIGH_PERIOD_US : bus->half_period_us);
}

static HAL_StatusTypeDef raise_scl(soft_i2c_t *bus)
{
    write_scl(bus, GPIO_PIN_SET);
    /* The normal path avoids a timer read at every clock edge. */
    if (HAL_GPIO_ReadPin(bus->scl_port, bus->scl_pin) == GPIO_PIN_SET) return HAL_OK;
    uint32_t start = Board_Micros();
    while (HAL_GPIO_ReadPin(bus->scl_port, bus->scl_pin) == GPIO_PIN_RESET) {
        if ((uint32_t)(Board_Micros()-start) >= APP_SOFT_I2C_TIMEOUT_US) {
            return HAL_TIMEOUT; /* A short or clock stretching cannot hang forever. */
        }
        Board_DelayUs(1U);
    }
    return HAL_OK;
}
void SoftI2C_Init(soft_i2c_t *bus)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    /* Set the output latch before enabling outputs, avoiding a low glitch. */
    write_scl(bus, GPIO_PIN_SET);
    write_sda(bus, GPIO_PIN_SET);
    gpio.Pin = bus->scl_pin;
    HAL_GPIO_Init(bus->scl_port, &gpio);
    gpio.Pin = bus->sda_pin;
    HAL_GPIO_Init(bus->sda_port, &gpio);
}
HAL_StatusTypeDef SoftI2C_Start(soft_i2c_t *bus)
{
    write_sda(bus, GPIO_PIN_SET);
    delay(bus);
    HAL_StatusTypeDef result = raise_scl(bus);
    if (result != HAL_OK) return result;
    delay(bus);
    if (read_sda(bus) == GPIO_PIN_RESET) return HAL_BUSY;
    write_sda(bus, GPIO_PIN_RESET); /* SDA falling while SCL is high. */
    delay(bus);
    write_scl(bus, GPIO_PIN_RESET);
    return HAL_OK;
}
HAL_StatusTypeDef SoftI2C_Stop(soft_i2c_t *bus)
{
    write_scl(bus, GPIO_PIN_RESET);
    write_sda(bus, GPIO_PIN_RESET);
    delay(bus);
    HAL_StatusTypeDef result = raise_scl(bus);
    if (result == HAL_OK) delay(bus);
    write_sda(bus, GPIO_PIN_SET); /* Release both lines even after a timeout. */
    delay(bus);
    return result;
}
HAL_StatusTypeDef SoftI2C_SendByte(soft_i2c_t *bus, uint8_t byte)
{
    for (unsigned bit = 0; bit < 8U; ++bit) {
        write_sda(bus, (byte & 0x80U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay(bus);
        HAL_StatusTypeDef result = raise_scl(bus);
        if (result != HAL_OK) return result;
        high_delay(bus);
        write_scl(bus, GPIO_PIN_RESET);
        byte = (uint8_t)(byte << 1U);
    }
    write_sda(bus, GPIO_PIN_SET);
    return HAL_OK;
}
HAL_StatusTypeDef SoftI2C_ReceiveByte(soft_i2c_t *bus, uint8_t *byte)
{
    if (byte == NULL) return HAL_ERROR;
    uint8_t value = 0U;
    write_sda(bus, GPIO_PIN_SET);
    for (unsigned bit = 0; bit < 8U; ++bit) {
        delay(bus);
        HAL_StatusTypeDef result = raise_scl(bus);
        if (result != HAL_OK) return result;
        high_delay(bus);
        value = (uint8_t)((value << 1U) | (read_sda(bus) == GPIO_PIN_SET ? 1U : 0U));
        write_scl(bus, GPIO_PIN_RESET);
    }
    *byte = value;
    return HAL_OK;
}
HAL_StatusTypeDef SoftI2C_SendAck(soft_i2c_t *bus, uint8_t ack_bit)
{
    write_sda(bus, ack_bit != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay(bus);
    HAL_StatusTypeDef result = raise_scl(bus);
    if (result == HAL_OK) high_delay(bus);
    write_scl(bus, GPIO_PIN_RESET);
    write_sda(bus, GPIO_PIN_SET);
    return result;
}
HAL_StatusTypeDef SoftI2C_ReceiveAck(soft_i2c_t *bus, uint8_t *ack_bit)
{
    if (ack_bit == NULL) return HAL_ERROR;
    write_sda(bus, GPIO_PIN_SET);
    delay(bus);
    HAL_StatusTypeDef result = raise_scl(bus);
    if (result != HAL_OK) return result;
    high_delay(bus);
    *ack_bit = read_sda(bus) == GPIO_PIN_SET ? 1U : 0U;
    write_scl(bus, GPIO_PIN_RESET);
    return HAL_OK;
}
static HAL_StatusTypeDef send_checked(soft_i2c_t *bus, uint8_t byte)
{
    HAL_StatusTypeDef result = SoftI2C_SendByte(bus, byte);
    if (result != HAL_OK) return result;
    uint8_t ack;
    result = SoftI2C_ReceiveAck(bus, &ack);
    return result != HAL_OK ? result : (ack == 0U ? HAL_OK : HAL_ERROR);
}
static HAL_StatusTypeDef finish(soft_i2c_t *bus, HAL_StatusTypeDef result)
{
    HAL_StatusTypeDef stopped = SoftI2C_Stop(bus);
    return result == HAL_OK ? stopped : result;
}
static HAL_StatusTypeDef begin_write(soft_i2c_t *bus, uint8_t address)
{
    HAL_StatusTypeDef result = SoftI2C_Start(bus);
    return result == HAL_OK ? send_checked(bus, (uint8_t)(address << 1U)) : result;
}
HAL_StatusTypeDef SoftI2C_Probe(soft_i2c_t *bus, uint8_t address)
{
    if (bus == NULL || address > 0x7FU) return HAL_ERROR;
    return finish(bus, begin_write(bus, address));
}
HAL_StatusTypeDef SoftI2C_Write(soft_i2c_t *bus, uint8_t address, const uint8_t *data, uint16_t size)
{
    if (bus == NULL || address > 0x7FU || (data == NULL && size != 0U)) return HAL_ERROR;
    HAL_StatusTypeDef result = begin_write(bus, address);
    for (uint16_t i = 0; result == HAL_OK && i < size; ++i) result = send_checked(bus, data[i]);
    return finish(bus, result);
}
HAL_StatusTypeDef SoftI2C_WriteReg(soft_i2c_t *bus, uint8_t address, uint8_t reg,
                                 const uint8_t *data, uint16_t size)
{
    if (bus == NULL || address > 0x7FU || (data == NULL && size != 0U)) return HAL_ERROR;
    HAL_StatusTypeDef result = begin_write(bus, address);
    if (result == HAL_OK) result = send_checked(bus, reg);
    for (uint16_t i = 0; result == HAL_OK && i < size; ++i) result = send_checked(bus, data[i]);
    return finish(bus, result);
}
HAL_StatusTypeDef SoftI2C_ReadReg(soft_i2c_t *bus, uint8_t address, uint8_t reg,
                                uint8_t *data, uint16_t size)
{
    if (bus == NULL || address > 0x7FU || data == NULL || size == 0U) return HAL_ERROR;
    HAL_StatusTypeDef result = begin_write(bus, address);
    if (result == HAL_OK) result = send_checked(bus, reg);
    if (result == HAL_OK) result = SoftI2C_Start(bus); /* Repeated START, not STOP. */
    if (result == HAL_OK) result = send_checked(bus, (uint8_t)((address << 1U) | 1U));
    for (uint16_t i = 0; result == HAL_OK && i < size; ++i) {
        result = SoftI2C_ReceiveByte(bus, &data[i]);
        if (result == HAL_OK) result = SoftI2C_SendAck(bus, i+1U == size ? 1U : 0U);
    }
    return finish(bus, result);
}
HAL_StatusTypeDef SoftI2C_Recover(soft_i2c_t *bus)
{
    write_sda(bus, GPIO_PIN_SET);
    HAL_StatusTypeDef result = raise_scl(bus);
    if (result != HAL_OK) return finish(bus, result);
    delay(bus);
    for (unsigned i = 0; i < 9U && read_sda(bus) == GPIO_PIN_RESET; ++i) {
        write_scl(bus, GPIO_PIN_RESET);
        delay(bus);
        result = raise_scl(bus);
        if (result != HAL_OK) return finish(bus, result);
        delay(bus);
    }
    result = SoftI2C_Stop(bus);
    if (result != HAL_OK) return result;
    return read_sda(bus) == GPIO_PIN_SET ? HAL_OK : HAL_BUSY;
}
