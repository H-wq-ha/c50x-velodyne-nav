/* Host regression: sample updates between I2C transactions must not tear words. */
#include "mpu6050.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t device_registers[256];
static uint8_t frames[2][14];
static unsigned frame_index, sample_read_calls, last_read_size;
static bool update_after_transaction;
static HAL_StatusTypeDef acquisition_status = HAL_OK;
static uint32_t clock_us;

void HAL_Delay(uint32_t milliseconds) { (void)milliseconds; }
uint32_t Board_Micros(void) { return ++clock_us; }
HAL_StatusTypeDef SoftI2C_WriteReg(soft_i2c_t *bus, uint8_t address, uint8_t reg,
                                 const uint8_t *data, uint16_t size)
{
    (void)bus;
    assert(address == 0x68U);
    assert(size == 1U);
    device_registers[reg] = data[0];
    return HAL_OK;
}
HAL_StatusTypeDef SoftI2C_ReadReg(soft_i2c_t *bus, uint8_t address, uint8_t reg,
                                uint8_t *data, uint16_t size)
{
    (void)bus;
    assert(address == 0x68U);
    if (reg >= 0x3BU && reg <= 0x48U) {
        assert((unsigned)(reg - 0x3BU) + size <= 14U);
        ++sample_read_calls;
        last_read_size = size;
        /* A failed transaction may have already written a partial buffer. */
        unsigned copied = acquisition_status == HAL_OK ? size : (size > 4U ? 4U : size);
        memcpy(data, frames[frame_index] + reg - 0x3BU, copied);
        if (update_after_transaction) frame_index ^= 1U;
        return acquisition_status;
    }
    assert(size == 1U);
    data[0] = device_registers[reg];
    return HAL_OK;
}
static void put_word(uint8_t *buffer, int16_t value)
{
    uint16_t bits = (uint16_t)value;
    buffer[0] = (uint8_t)(bits >> 8U);
    buffer[1] = (uint8_t)bits;
}
static void fill_frame(uint8_t *buffer, const int16_t words[7])
{
    for (unsigned i = 0; i < 7U; ++i) put_word(buffer + 2U*i, words[i]);
}
int main(void)
{
    soft_i2c_t bus = {0};
    mpu6050_t imu = {.bus = &bus, .address = 0x68U, .who_am_i = 0x68U};
    /* Signed limits, negative temperature, and the 0x00FF -> 0x0100 boundary. */
    const int16_t old_words[7] = {255, -1, INT16_MIN, -1234, INT16_MAX, INT16_MIN, 256};
    const int16_t new_words[7] = {256, 0, INT16_MAX, 1234, 0, -1, 1};
    fill_frame(frames[0], old_words);
    fill_frame(frames[1], new_words);
    update_after_transaction = true;
    mpu6050_raw_t raw;
    assert(MPU6050_GetData(&imu, &raw) == HAL_OK);
    assert(sample_read_calls == 1U && last_read_size == 14U);
    assert(raw.accel[0] == 255 && raw.accel[1] == -1 && raw.accel[2] == INT16_MIN);
    assert(raw.temperature == -1234);
    assert(raw.gyro[0] == INT16_MAX && raw.gyro[1] == INT16_MIN && raw.gyro[2] == 256);
    assert(MPU6050_GetData(&imu, &raw) == HAL_OK);
    assert(sample_read_calls == 2U && raw.accel[0] == 256 && raw.temperature == 1234);
    assert(raw.accel[2] == INT16_MAX && raw.gyro[1] == -1);

    /* Partial reads propagate HAL status and preserve the last good output. */
    mpu6050_raw_t saved = raw;
    acquisition_status = HAL_TIMEOUT;
    assert(MPU6050_GetData(&imu, &raw) == HAL_TIMEOUT);
    assert(memcmp(&raw, &saved, sizeof(raw)) == 0);
    assert(imu.last_status == HAL_TIMEOUT && imu.last_register == 0x3BU);
    assert(strcmp(imu.stage, "READ_BURST") == 0);
    imu_sample_t sample;
    memset(&sample, 0x5A, sizeof(sample));
    imu_sample_t saved_sample = sample;
    assert(MPU6050_Read(&imu, &sample) == IMU_READ_ERROR);
    assert(memcmp(&sample, &saved_sample, sizeof(sample)) == 0);
    unsigned before_invalid = sample_read_calls;
    assert(MPU6050_GetData(&imu, NULL) == HAL_ERROR);
    assert(sample_read_calls == before_invalid);

    /* Physical SI conversion after burst decoding, including both temperature models. */
    acquisition_status = HAL_OK;
    frame_index = 0;
    update_after_transaction = false;
    const int16_t physical_words[7] = {8192, -8192, 16384, 340, 164, -164, 0};
    fill_frame(frames[0], physical_words);
    imu.who_am_i = 0x68U;
    assert(MPU6050_Read(&imu, &sample) == IMU_READ_OK);
    assert(fabsf(sample.accel_mps2[0] - 9.80665f) < 1e-5f);
    assert(fabsf(sample.accel_mps2[1] + 9.80665f) < 1e-5f);
    assert(fabsf(sample.accel_mps2[2] - 19.6133f) < 1e-5f);
    assert(fabsf(sample.gyro_rads[0] - 0.174532925f) < 1e-6f);
    assert(fabsf(sample.gyro_rads[1] + 0.174532925f) < 1e-6f);
    assert(fabsf(sample.temperature_c - 37.53f) < 1e-5f);
    assert(sample.monotonic_us == saved_sample.monotonic_us);
    assert(sample.epoch_us == saved_sample.epoch_us);
    imu.who_am_i = 0x70U;
    assert(MPU6050_Read(&imu, &sample) == IMU_READ_OK);
    assert(fabsf(sample.temperature_c - (21.0f + 340.0f/333.87f)) < 1e-5f);

    assert(MPU6050_EnableDataReady(&imu));
    assert(device_registers[0x37] == 0U && device_registers[0x38] == 1U);

    /* Initialization still selects the requested range and DLPF for both chips. */
    for (unsigned chip = 0; chip < 2U; ++chip) {
        device_registers[0x75] = chip == 0U ? 0x68U : 0x70U;
        assert(MPU6050_Init(&imu, &bus));
        assert(device_registers[0x1A] == 0x03U);
        assert(device_registers[0x1C] == 0x08U);
        assert(device_registers[0x1B] == 0x18U);
        assert(device_registers[0x19] == 0x04U);
        if (chip == 1U) assert(device_registers[0x1D] == 0x03U);
    }
    puts("PASS: coherent burst, signed boundaries, partial-read preservation, SI units, both chip configurations");
    return 0;
}
