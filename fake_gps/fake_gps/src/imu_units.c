#include "imu_units.h"
#include <stdio.h>

static int32_t signed_be16(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] << 8U) | p[1];
    return u < 32768U ? (int32_t)u : (int32_t)u - 65536;
}
void IMU_ConvertRaw(const int16_t accel[3], const int16_t gyro[3], int16_t temperature,
                    float accel_lsb_per_g, float gyro_lsb_per_dps,
                    float temp_lsb_per_c, float temp_offset_c, imu_sample_t *sample)
{
    const float accel_scale = 9.80665f / accel_lsb_per_g;
    const float gyro_scale = 0.017453292519943295f / gyro_lsb_per_dps;
    for (unsigned axis = 0; axis < 3U; ++axis) {
        sample->accel_mps2[axis] = (float)accel[axis] * accel_scale;
        sample->gyro_rads[axis] = (float)gyro[axis] * gyro_scale;
    }
    sample->temperature_c = (float)temperature/temp_lsb_per_c + temp_offset_c;
}
void IMU_DecodeBurst(const uint8_t registers[14], imu_sample_t *sample)
{
    int16_t accel[3], gyro[3];
    for (unsigned axis = 0; axis < 3U; ++axis) {
        accel[axis] = (int16_t)signed_be16(registers+2U*axis);
        gyro[axis] = (int16_t)signed_be16(registers+8U+2U*axis);
    }
    IMU_ConvertRaw(accel, gyro, (int16_t)signed_be16(registers+6U),
                   16384.0f, 131.0f, 340.0f, 36.53f, sample);
}
int IMU_FormatFixed(char *out, size_t capacity, float value, unsigned decimals)
{
    static const uint32_t scale[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    if (out == NULL || capacity == 0U || decimals > 6U) {
        return -1;
    }
    float magnitude = value < 0.0f ? -value : value;
    if (!(magnitude <= 2000.0f)) {
        return -1; /* Also rejects NaN and infinity. */
    }
    uint32_t rounded = (uint32_t)(magnitude*(float)scale[decimals] + 0.5f);
    int count;
    if (decimals == 0U) {
        count = snprintf(out, capacity, "%c%lu", value < 0.0f ? '-' : '+',
                         (unsigned long)rounded);
    } else {
        count = snprintf(out, capacity, "%c%lu.%0*lu", value < 0.0f ? '-' : '+',
                         (unsigned long)(rounded/scale[decimals]), (int)decimals,
                         (unsigned long)(rounded%scale[decimals]));
    }
    return count >= 0 && (size_t)count < capacity ? count : -1;
}
