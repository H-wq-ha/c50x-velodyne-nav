#include "mpu6050.h"
#include <stddef.h>

enum {
    SMPLRT_DIV = 0x19, CONFIG = 0x1A, GYRO_CONFIG = 0x1B,
    ACCEL_CONFIG = 0x1C, ACCEL_CONFIG2 = 0x1D,
    ACCEL_XOUT_H = 0x3B,
    PWR_MGMT_1 = 0x6B, PWR_MGMT_2 = 0x6C, WHO_AM_I = 0x75,
};

HAL_StatusTypeDef MPU6050_WriteReg(mpu6050_t *imu, uint8_t reg, uint8_t data)
{
    imu->last_register = reg;
    imu->stage = "WRITE_REG";
    imu->last_status = SoftI2C_WriteReg(imu->bus, imu->address, reg, &data, 1U);
    return imu->last_status;
}
HAL_StatusTypeDef MPU6050_ReadReg(mpu6050_t *imu, uint8_t reg, uint8_t *data)
{
    /* 教程：Start -> D0 -> ACK -> Reg -> ACK -> Start -> D1 -> ACK
       -> ReceiveByte -> SendAck(1，即NACK) -> Stop，每次只读一个寄存器。 */
    imu->last_register = reg;
    imu->stage = "READ_REG";
    imu->last_status = SoftI2C_ReadReg(imu->bus, imu->address, reg, data, 1U);
    return imu->last_status;
}
HAL_StatusTypeDef MPU6050_GetID(mpu6050_t *imu, uint8_t *id)
{
    return MPU6050_ReadReg(imu, WHO_AM_I, id);
}
const char *MPU6050_ModelName(const mpu6050_t *imu)
{
    if (imu->who_am_i == 0x68U) return "MPU6050";
    if (imu->who_am_i == 0x70U) return "MPU6500";
    return "UNKNOWN";
}
static bool write_checked(mpu6050_t *imu, uint8_t reg, uint8_t value)
{
    uint8_t returned = 0U;
    if (MPU6050_WriteReg(imu, reg, value) != HAL_OK ||
        MPU6050_ReadReg(imu, reg, &returned) != HAL_OK) return false;
    if (returned != value) {
        imu->stage = "CONFIG_MISMATCH";
        imu->last_status = HAL_ERROR;
        return false;
    }
    return true;
}
bool MPU6050_Init(mpu6050_t *imu, soft_i2c_t *bus)
{
    imu->bus = bus;
    imu->who_am_i = 0U;
    imu->last_status = HAL_ERROR;
    imu->last_register = WHO_AM_I;
    imu->stage = "PROBE";
    for (unsigned i = 0; i < 2U; ++i) {
        imu->probes[i].attempted = false;
        imu->probes[i].who_am_i = 0U;
        imu->probes[i].status = HAL_ERROR;
    }
    bool found = false;
    for (unsigned i = 0; i < 2U; ++i) {
        imu->address = (uint8_t)(0x68U+i);
        imu_probe_t *probe = &imu->probes[i];
        probe->attempted = true;
        probe->status = MPU6050_GetID(imu, &probe->who_am_i);
        if (probe->status == HAL_OK &&
            (probe->who_am_i == 0x68U || probe->who_am_i == 0x70U)) {
            imu->who_am_i = probe->who_am_i;
            found = true;
            break;
        }
    }
    if (!found) {
        /* 不能把前一地址的ID配到后一地址。 */
        imu->stage = "NO_RESPONSE";
        for (unsigned i = 0; i < 2U; ++i) {
            if (imu->probes[i].status == HAL_OK) {
                imu->address = (uint8_t)(0x68U+i);
                imu->who_am_i = imu->probes[i].who_am_i;
                imu->last_status = HAL_OK;
                imu->stage = "UNSUPPORTED_ID";
                break;
            }
        }
        return false;
    }
    /* 复位/等待及配置读回用于本工程的错误诊断。 */
    if (MPU6050_WriteReg(imu, PWR_MGMT_1, 0x80U) != HAL_OK) return false;
    HAL_Delay(100U);
    /* 唤醒、所有轴使能、分频、低通3档、陀螺仪量程3档、加速度量程1档。 */
    static const uint8_t configuration[][2] = {
        {PWR_MGMT_1, 0x01U},
        {PWR_MGMT_2, 0x00U},
        {SMPLRT_DIV, (1000U/APP_IMU_RATE_HZ)-1U}, /* 默认0x04，200Hz。 */
        {CONFIG, 0x03U},      /* DLPF3：MPU6050加速度44Hz/陀螺42Hz；MPU6500陀螺41Hz。 */
        {GYRO_CONFIG, 0x18U},  /* +/-2000deg/s，16.4LSB/(deg/s)。 */
        {ACCEL_CONFIG, 0x08U}, /* 加速度量程1档：+/-4g，8192LSB/g。 */
    };
    for (unsigned i = 0; i < sizeof(configuration)/sizeof(configuration[0]); ++i) {
        if (!write_checked(imu, configuration[i][0], configuration[i][1])) return false;
    }
    /* MPU6500有独立的加速度低通配置及不同的温度参数。 */
    if (imu->who_am_i == 0x70U && !write_checked(imu, ACCEL_CONFIG2, 0x03U)) return false;
    HAL_Delay(50U);
    imu->stage = "READY";
    return true;
}
static int16_t decode_signed_be16(const uint8_t *bytes)
{
    uint32_t bits = ((uint32_t)bytes[0] << 8U) | bytes[1];
    return (int16_t)(bits < 32768U ? (int32_t)bits : (int32_t)bits-65536);
}
HAL_StatusTypeDef MPU6050_GetData(mpu6050_t *imu, mpu6050_raw_t *raw)
{
    if (raw == NULL) {
        imu->stage = "INVALID_ARGUMENT";
        imu->last_status = HAL_ERROR;
        return HAL_ERROR;
    }
    uint8_t registers[14];
    imu->last_register = ACCEL_XOUT_H;
    imu->stage = "READ_BURST";
    /* 一次I2C事务读取0x3B..0x48，保证高低字节及六轴属于同一采样时刻。
       SoftI2C_ReadReg使用Repeated START，前13字节ACK，最后字节NACK后STOP。 */
    imu->last_status = SoftI2C_ReadReg(imu->bus, imu->address, ACCEL_XOUT_H,
                                      registers, sizeof(registers));
    if (imu->last_status != HAL_OK) return imu->last_status;

    mpu6050_raw_t measured;
    for (unsigned axis = 0; axis < 3U; ++axis) {
        measured.accel[axis] = decode_signed_be16(registers + 2U*axis);
        measured.gyro[axis] = decode_signed_be16(registers + 8U + 2U*axis);
    }
    measured.temperature = decode_signed_be16(registers + 6U);
    *raw = measured; /* 全部成功后才提交，失败时不发送半份新数据。 */
    imu->stage = "READ_OK";
    return HAL_OK;
}
imu_read_result_t MPU6050_Read(mpu6050_t *imu, imu_sample_t *sample)
{
    mpu6050_raw_t raw;
    uint32_t acquisition_time = Board_Micros();
    if (MPU6050_GetData(imu, &raw) != HAL_OK) return IMU_READ_ERROR;
    IMU_ConvertRaw(raw.accel, raw.gyro, raw.temperature,
                   8192.0f, 16.4f,
                   imu->who_am_i == 0x70U ? 333.87f : 340.0f,
                   imu->who_am_i == 0x70U ? 21.0f : 36.53f, sample);
    sample->timestamp_us = acquisition_time; /* 本轮读取开始时间，非同步转换时刻。 */
    return IMU_READ_OK;
}
