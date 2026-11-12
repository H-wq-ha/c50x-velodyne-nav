#include "imu_protocol.h"
#include <stdio.h>
#include <string.h>
_Static_assert(sizeof(float) == 4U, "Protocol requires 32-bit float.");

uint16_t IMU_Crc16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (unsigned bit = 0; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 0x8000U) != 0U
                           ? ((uint32_t)crc << 1U) ^ 0x1021U : (uint32_t)crc << 1U);
        }
    }
    return crc;
}
static void put_u32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4U; ++i) {
        p[i] = (uint8_t)(value >> (8U*i));
    }
}
static void put_f32(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(p, bits);
}
void IMU_PackBinary(const imu_sample_t *sample, uint8_t out[IMU_PACKET_SIZE])
{
    out[0] = 0xFFU;
    uint8_t *payload = out + 1U;
    payload[0] = 1U;
    payload[1] = 1U;
    put_u32(payload + 2U, sample->sequence);
    put_u32(payload + 6U, sample->timestamp_us);
    for (unsigned axis = 0; axis < 3U; ++axis) {
        put_f32(payload + 10U + 4U*axis, sample->accel_mps2[axis]);
        put_f32(payload + 22U + 4U*axis, sample->gyro_rads[axis]);
    }
    put_f32(payload + 34U, sample->temperature_c);
    uint16_t crc = IMU_Crc16(payload, 38U);
    payload[38] = (uint8_t)crc;
    payload[39] = (uint8_t)(crc >> 8U);
    out[41] = 0xFEU;
}
int IMU_PackText(const imu_sample_t *sample, char *out, size_t capacity)
{
    char values[7][16];
    for (unsigned axis = 0; axis < 3U; ++axis) {
        if (IMU_FormatFixed(values[axis], sizeof(values[axis]), sample->accel_mps2[axis], 3U) < 0 ||
            IMU_FormatFixed(values[axis+3U], sizeof(values[axis+3U]), sample->gyro_rads[axis], 6U) < 0) {
            return -1;
        }
    }
    if (IMU_FormatFixed(values[6], sizeof(values[6]), sample->temperature_c, 3U) < 0) {
        return -1;
    }
    int count = snprintf(out, capacity, "@IMU,%lu,%lu,%s,%s,%s,%s,%s,%s,%s\r\n",
                         (unsigned long)sample->sequence, (unsigned long)sample->timestamp_us,
                         values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
    return count >= 0 && (size_t)count < capacity ? count : -1;
}
