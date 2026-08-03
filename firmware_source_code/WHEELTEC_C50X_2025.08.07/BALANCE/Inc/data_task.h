#ifndef __DATA_TASK_H
#define __DATA_TASK_H

#include "system.h"

#define DATA_TASK_PRIO     4
#define DATA_STK_SIZE      512
#define DATA_TASK_RATE     RATE_200_HZ

#define FRAME_HEADER       0X7B
#define FRAME_TAIL         0X7D
#define SEND_DATA_SIZE     24

#define AutoCharge_HEADER  0X7C
#define AutoCharge_TAIL    0X7F
#define AutoCharge_DATA_SIZE 8

typedef struct __Mpu6050_Data_
{
    short X_data;
    short Y_data;
    short Z_data;
} Mpu6050_Data;

typedef struct __Robot_Data_
{
    short X_speed;
    short Y_speed;
    short Z_speed;
} Robot_Vel;

typedef struct _SEND_DATA_
{
    struct _Sensor_Str_
    {
        unsigned char Frame_Header;
        Robot_Vel Vel;
        Mpu6050_Data Accelerometer;
        Mpu6050_Data Gyroscope;
        short Power_Voltage;
        unsigned char Frame_Tail;
    } Sensor_Str;

    unsigned char buffer[SEND_DATA_SIZE];
} SEND_DATA;

typedef struct _SEND_AutoCharge_DATA_
{
    unsigned char buffer[AutoCharge_DATA_SIZE];
    struct _AutoCharge_Str_
    {
        unsigned char Frame_Header;
        short Charging_Current;
        unsigned char RED;
        unsigned char Charging;
        unsigned char yuliu;
        unsigned char Frame_Tail;
    } AutoCharge_Str;
} SEND_AutoCharge_DATA;

uint8_t Check_BCC(const uint8_t *data, uint16_t length);
void data_task(void *pvParameters);

extern TaskHandle_t data_TaskHandle;

static float* Kinematics_akm_diff(float motorA, float motorB);
static float* Kinematics_omni(float motorA, float motorB, float motorC);
static float* Kinematics_mec_4wd(float motorA, float motorB, float motorC, float motorD);
static void data_transition(void);
static void Usart1_SendTask(void);
static void Usart3_SendTask(void);
static void CAN1_SendTask(void);

#endif
