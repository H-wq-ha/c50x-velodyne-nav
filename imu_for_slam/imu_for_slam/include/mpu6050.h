#ifndef MPU6050_H
#define MPU6050_H
#include "board.h"
#include "imu_units.h"
typedef struct {
    bool attempted;
    uint8_t who_am_i;
    HAL_StatusTypeDef status;
} imu_probe_t;
typedef struct {
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temperature;
} mpu6050_raw_t;
typedef struct {
    soft_i2c_t *bus;
    uint8_t address;
    uint8_t who_am_i;
    HAL_StatusTypeDef last_status;
    uint8_t last_register;
    const char *stage;
    imu_probe_t probes[2]; /* Index 0: address 0x68; index 1: address 0x69. */
} mpu6050_t;
typedef enum { IMU_READ_OK, IMU_READ_ERROR } imu_read_result_t;
bool MPU6050_Init(mpu6050_t *imu, soft_i2c_t *bus);
HAL_StatusTypeDef MPU6050_WriteReg(mpu6050_t *imu, uint8_t reg, uint8_t data);
HAL_StatusTypeDef MPU6050_ReadReg(mpu6050_t *imu, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MPU6050_GetID(mpu6050_t *imu, uint8_t *id);
/* 从0x3B一次连续读取14字节；读取失败时不覆盖raw中的上一份数据。 */
HAL_StatusTypeDef MPU6050_GetData(mpu6050_t *imu, mpu6050_raw_t *raw);
const char *MPU6050_ModelName(const mpu6050_t *imu);
imu_read_result_t MPU6050_Read(mpu6050_t *imu, imu_sample_t *sample);
#endif
