#ifndef IMU_UNITS_H
#define IMU_UNITS_H
#include <stddef.h>
#include <stdint.h>
typedef struct {
    uint32_t sequence;
    uint32_t timestamp_us;
    float accel_mps2[3]; /* Specific force; no gravity or bias subtraction. */
    float gyro_rads[3];
    float temperature_c;
} imu_sample_t;
/* 按实际配置的灵敏度转换，支持不同量程和型号的温度参数。 */
void IMU_ConvertRaw(const int16_t accel[3], const int16_t gyro[3], int16_t temperature,
                    float accel_lsb_per_g, float gyro_lsb_per_dps,
                    float temp_lsb_per_c, float temp_offset_c, imu_sample_t *sample);
/* 旧版14字节解码工具：仅适用MPU6050的+/-2g、+/-250deg/s。
   当前驱动使用GetData连续读取14字节并解析，再按实际量程调用IMU_ConvertRaw。 */
void IMU_DecodeBurst(const uint8_t registers[14], imu_sample_t *sample);
int IMU_FormatFixed(char *out, size_t capacity, float value, unsigned decimals);
#endif
