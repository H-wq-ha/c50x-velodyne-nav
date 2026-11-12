#include "imu_protocol.h"
#include <string.h>
_Static_assert(sizeof(float) == 4U, "Protocol requires binary32 floats");

uint16_t IMU_Crc16(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (unsigned bit = 0U; bit < 8U; ++bit)
            crc = (uint16_t)((crc & 0x8000U) ? ((uint32_t)crc << 1U) ^ 0x1021U : (uint32_t)crc << 1U);
    }
    return crc;
}
static void put32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; ++i) p[i] = (uint8_t)(value >> (8U * i));
}
static void put64(uint8_t *p, uint64_t value)
{
    put32(p, (uint32_t)value);
    put32(p + 4U, (uint32_t)(value >> 32U));
}
static void putfloat(uint8_t *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, 4U);
    put32(p, bits);
}
static void begin(uint8_t *out, uint8_t type)
{
    out[0] = 0xA5U;
    out[1] = 0x5AU;
    out[2] = 2U;
    out[3] = type;
    out[4] = IMU_PAYLOAD_SIZE;
    out[5] = 0U;
}
static void finish(uint8_t *out)
{
    uint16_t crc = IMU_Crc16(out + 2U, IMU_PACKET_SIZE - 4U);
    out[70] = (uint8_t)crc;
    out[71] = (uint8_t)(crc >> 8U);
}
void IMU_PackBinary(const imu_sample_t *sample, uint8_t out[IMU_PACKET_SIZE])
{
    begin(out, 1U);
    uint8_t *p = out + 6U;
    put32(p, sample->sequence);
    put64(p + 4U, sample->monotonic_us);
    put64(p + 12U, sample->epoch_us);
    put32(p + 20U, sample->hour_us);
    put32(p + 24U, sample->pps_count);
    for (unsigned axis = 0U; axis < 3U; ++axis) {
        putfloat(p + 28U + 4U * axis, sample->accel_mps2[axis]);
        putfloat(p + 40U + 4U * axis, sample->gyro_rads[axis]);
    }
    putfloat(p + 52U, sample->temperature_c);
    put32(p + 56U, sample->flags);
    put32(p + 60U, sample->lost_samples);
    finish(out);
}
void IMU_PackStatus(const uint32_t words[16], uint8_t out[IMU_PACKET_SIZE])
{
    begin(out, 2U);
    for (unsigned i = 0U; i < 16U; ++i) put32(out + 6U + i * 4U, words[i]);
    finish(out);
}
