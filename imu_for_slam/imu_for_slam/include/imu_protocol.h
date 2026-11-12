#ifndef IMU_PROTOCOL_H
#define IMU_PROTOCOL_H
#include "imu_units.h"
#define IMU_PACKET_SIZE 42U
#define IMU_PAYLOAD_SIZE 40U
#define IMU_TEXT_CAPACITY 160U
uint16_t IMU_Crc16(const uint8_t *data, size_t size);
void IMU_PackBinary(const imu_sample_t *sample, uint8_t out[IMU_PACKET_SIZE]);
int IMU_PackText(const imu_sample_t *sample, char *out, size_t capacity);
#endif
