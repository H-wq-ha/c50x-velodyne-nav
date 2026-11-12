#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H
#include "imu_units.h"
#define IMU_PACKET_SIZE 72U
#define IMU_PAYLOAD_SIZE 64U
enum {
    IMU_FLAG_TIME_VALID = 1U, IMU_FLAG_DRDY_CAPTURE = 2U,
    IMU_FLAG_HSE = 4U, IMU_FLAG_MPU6500 = 8U, IMU_FLAG_GAP = 16U,
    IMU_FLAG_ONLINE = 32U, IMU_FLAG_STREAMING = 64U
};
uint16_t IMU_Crc16(const uint8_t *data, size_t size);
void IMU_PackBinary(const imu_sample_t *sample, uint8_t out[IMU_PACKET_SIZE]);
void IMU_PackStatus(const uint32_t words[16], uint8_t out[IMU_PACKET_SIZE]);
#endif
