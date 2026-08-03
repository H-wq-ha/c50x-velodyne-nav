#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#include "system.h"

#define IMU_TASK_PRIO      3
#define IMU_STK_SIZE       256
#define IMU_TASK_RATE      RATE_200_HZ

typedef struct{
	short x;
	short y;
	short z;
}IMU_BASE_t;

//imu��������,����ƫ��ֵ��ԭʼֵ
typedef struct{
	IMU_BASE_t Deviation_accel;//ƫ��ֵ,���ھ������.��ֵ�ɼ��󲻻ᷢ���ı�
	IMU_BASE_t Deviation_gyro;
	IMU_BASE_t Original_accel;
	IMU_BASE_t Original_gyro;
	IMU_BASE_t gyro;
	IMU_BASE_t accel;
}IMU_DATA_t;

extern IMU_DATA_t imu;

//����ӿ�
void MPU6050_task(void *pvParameters);
void ICM20948_task(void *pvParameters);

void ImuData_copy(IMU_BASE_t* req_val,const IMU_BASE_t* copied_val);

#endif
