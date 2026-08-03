#include "MPU6050.h"
#include "imu_task.h"
#include "I2C.h"

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Ax, Ay, Az: The original reading of the accelerometer on the x,y, and z axes (plus or minus)
Output  : 0: success, others: error code
�������ܣ��õ����ٶ�ֵ(ԭʼֵ)
**************************************************************************/
void MPU6050_Get_Accelscope(void)
{
    imu.accel.x =(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
    imu.accel.y =(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
    imu.accel.z =(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�

    if(SysVal.Time_count<CONTROL_DELAY) // 10 seconds before starting //����ǰ10��
    {
		
    }
    else //10 seconds after starting //����10���
    {
        //Save the raw data to update zero by clicking the user button
        //����ԭʼ�������ڵ����û������������
		imu.Original_accel.x = imu.accel.x;
		imu.Original_accel.y = imu.accel.y;
		imu.Original_accel.z = imu.accel.z;

        //Removes zero drift data
        //ȥ�����Ư�Ƶ�����
        imu.accel.x = imu.Original_accel.x - imu.Deviation_accel.x ;
        imu.accel.y = imu.Original_accel.y - imu.Deviation_accel.y ;
        imu.accel.z = imu.Original_accel.z - imu.Deviation_accel.z + 16384;
    }

}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : Gx, Gy, Gz: raw readings (plus or minus) of the x,y, and z axes of the gyroscope
Output  : 0: success, others: error code
�������ܣ����������ֵ(ԭʼֵ)
**************************************************************************/
void MPU6050_Get_Gyroscope(void)
{
    imu.gyro.x = (I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
    imu.gyro.y = (I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
    imu.gyro.z = (I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������

    if(SysVal.Time_count<CONTROL_DELAY) // 10 seconds before starting //����ǰ10��
    {

    }
    else //10 seconds after starting //����10���
    {

        //Save the raw data to update zero by clicking the user button
        //����ԭʼ�������ڵ����û������������
		imu.Original_gyro.x = imu.gyro.x;
		imu.Original_gyro.y = imu.gyro.y;
		imu.Original_gyro.z = imu.gyro.z;

        //Removes zero drift data
        //ȥ�����Ư�Ƶ�����
		imu.gyro.x = imu.Original_gyro.x - imu.Deviation_gyro.x;
		imu.gyro.y = imu.Original_gyro.y - imu.Deviation_gyro.y;
		imu.gyro.z = imu.Original_gyro.z - imu.Deviation_gyro.z;
    }
}

/**************************************************************************
Function: The new ADC data is updated to FIFO array for filtering
Input   : ax��ay��az��x��y, z-axis acceleration data��gx��gy��gz��x. Y, z-axis angular acceleration data
Output  : none
�������ܣ����µ�ADC���ݸ��µ� FIFO���飬�����˲�����
��ڲ�����ax��ay��az��x��y��z����ٶ����ݣ�gx��gy��gz��x��y��z��Ǽ��ٶ�����
����  ֵ����
**************************************************************************/
int16_t  MPU6050_FIFO[6][11];
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
    unsigned char i ;
    int32_t sum=0;
    for(i=1; i<10; i++) {	//FIFO ����
        MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
        MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
        MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
        MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
        MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
        MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
    MPU6050_FIFO[1][9]=ay;
    MPU6050_FIFO[2][9]=az;
    MPU6050_FIFO[3][9]=gx;
    MPU6050_FIFO[4][9]=gy;
    MPU6050_FIFO[5][9]=gz;

    sum=0;
    for(i=0; i<10; i++) {	//��ǰ����ĺϣ���ȡƽ��ֵ
        sum+=MPU6050_FIFO[0][i];
    }
    MPU6050_FIFO[0][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[1][i];
    }
    MPU6050_FIFO[1][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[2][i];
    }
    MPU6050_FIFO[2][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[3][i];
    }
    MPU6050_FIFO[3][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[4][i];
    }
    MPU6050_FIFO[4][10]=sum/10;

    sum=0;
    for(i=0; i<10; i++) {
        sum+=MPU6050_FIFO[5][i];
    }
    MPU6050_FIFO[5][10]=sum/10;
}

/**************************************************************************
Function: Setting the clock source of mpu6050
Input   : source��Clock source number
Output  : none
�������ܣ�����  MPU6050 ��ʱ��Դ
��ڲ�����source��ʱ��Դ���
����  ֵ����
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
**************************************************************************/
void MPU6050_setClockSource(uint8_t source) {
    I2C_WriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Setting the maximum range of mpu6050 accelerometer
Input   : range��Acceleration maximum range number
Output  : none
�������ܣ����� MPU6050 ���ٶȼƵ��������
��ڲ�����range�����ٶ�������̱��
����  ֵ����
**************************************************************************/
//#define MPU6050_ACCEL_FS_2          0x00  		//===�������+-2G
//#define MPU6050_ACCEL_FS_4          0x01			//===�������+-4G
//#define MPU6050_ACCEL_FS_8          0x02			//===�������+-8G
//#define MPU6050_ACCEL_FS_16         0x03			//===�������+-16G
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    I2C_WriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************************************************************
Function: Set mpu6050 to sleep mode or not
Input   : enable��1��sleep��0��work��
Output  : none
�������ܣ����� MPU6050 �Ƿ����˯��ģʽ
��ڲ�����enable��1��˯����0��������
����  ֵ����
**************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************************************************************
Function: Read identity
Input   : none
Output  : 0x68
�������ܣ���ȡ  MPU6050 WHO_AM_I ��ʶ
��ڲ�������
����  ֵ��0x68
**************************************************************************/
uint8_t MPU6050_getDeviceID(void) {
    return I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
}

/**************************************************************************
Function: Check whether mpu6050 is connected
Input   : none
Output  : 1��Connected��0��Not connected
�������ܣ����MPU6050 �Ƿ��Ѿ�����
��ڲ�������
����  ֵ��1�������ӣ�0��δ����
**************************************************************************/
uint8_t MPU6050_testConnection(void) {
    if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
        return 1;
    else return 0;
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable��1��yes��0;not
Output  : none
�������ܣ����� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
��ڲ�����enable��1���ǣ�0����
����  ֵ����
**************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************************************************************
Function: Setting whether mpu6050 is the host of aux I2C cable
Input   : enable��1��yes��0;not
Output  : none
�������ܣ����� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
��ڲ�����enable��1���ǣ�0����
����  ֵ����
**************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    I2C_WriteOneBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************************************************************
Function: Read mpu6050 built-in temperature sensor data
Input   : none
Output  : Centigrade temperature
�������ܣ���ȡMPU6050�����¶ȴ���������
��ڲ�������
����  ֵ�������¶�
**************************************************************************/
int Read_Temperature(void)
{
    float Temp;
    Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
    if(Temp>32768) Temp-=65536;	//��������ת��
    Temp=(36.53f+Temp/340)*10;	  //�¶ȷŴ�ʮ�����
    return (int)Temp;
}

/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : LPF: Digital low-pass filtering frequency (Hz)
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPUrobot_select_init.h�����ֵ�ͨ�˲���
��ڲ�����lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
unsigned char MPU6050_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6;
    return I2C_WriteOneByte(devAddr,MPU6050_RA_CONFIG,data); //Set the digital lowpass filter//�������ֵ�ͨ�˲���
}
/**************************************************************************
Function: Initialize TIM2 as the encoder interface mode
Input   : rate:4~1000(Hz)
Output  : 0: Settings successful, others: Settings failed
�������ܣ�����MPUrobot_select_init.h�Ĳ�����(�ٶ�Fs=1KHz)
��ڲ�����rate:4~1000(Hz)
����  ֵ��0:���óɹ�, ����:����ʧ��
**************************************************************************/
unsigned char MPU6050_Set_Rate(u16 rate)
{
    u8 data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=I2C_WriteOneByte(devAddr,MPU6050_RA_SMPLRT_DIV,data);	//Set the digital lowpass filter//�������ֵ�ͨ�˲���
    return MPU6050_Set_LPF(rate/2);	//Automatically sets LPF to half of the sampling rate //�Զ�����LPFΪ�����ʵ�һ��
}

/**************************************************************************
Function: initialization Mpu6050 to enter the available state
Input   : none
Output  : none
�������ܣ���ʼ��	MPU6050 �Խ������״̬
��ڲ�������
����  ֵ����
**************************************************************************/
u8 MPU6050_initialize(void) {
//    MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //����ʱ��
//    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//��������������
//    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//���ٶȶ�������� +-2G
//    MPU6050_setSleepEnabled(0); //���빤��״̬
//	  MPU6050_setI2CMasterModeEnabled(0);	 //����MPU6050 ����AUXI2C
//	  MPU6050_setI2CBypassEnabled(0);	 //����������I2C��	MPU6050��AUXI2C	ֱͨ�ر�

    u8 res;
    //IIC_Init();  //Initialize the IIC bus //��ʼ��IIC����
    I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X80);	//Reset MPUrobot_select_init.h //��λMPUrobot_select_init.h
    delay_ms(200); //Delay 200 ms //��ʱ200ms
    I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X00);	//Wake mpurobot_select_init.h //����MPUrobot_select_init.h

    //MPU6050_Set_Gyro_Fsr(1);  //Gyroscope sensor              //�����Ǵ�����,��500dps=��500��/s ��32768 (gyro/32768*500)*PI/180(rad/s)=gyro/3754.9(rad/s)
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    //MPU6050_Set_Accel_Fsr(0);	//Acceleration sensor           //���ٶȴ�����,��2g=��2*9.8m/s^2 ��32768 accel/32768*19.6=accel/1671.84
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    MPU6050_Set_Rate(200);			//Set the sampling rate to 200Hz for FAST-LIO //���ò�����50Hz

    I2C_WriteOneByte(devAddr,MPU6050_RA_INT_ENABLE,0X00);	  //Turn off all interrupts //�ر������ж�
    I2C_WriteOneByte(devAddr,MPU6050_RA_USER_CTRL,0X00);	//The I2C main mode is off //I2C��ģʽ�ر�
    I2C_WriteOneByte(devAddr,MPU6050_RA_FIFO_EN,0X00);	  //Close the FIFO //�ر�FIFO
    //The INT pin is low, enabling bypass mode to read the magnetometer directly
    //INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
    I2C_WriteOneByte(devAddr,MPU6050_RA_INT_PIN_CFG,0X80);
    //Read the ID of MPU6050
    //��ȡMPU6050��ID
    res=I2C_ReadOneByte(devAddr,MPU6050_RA_WHO_AM_I);
    if(res==MPU6050_DEFAULT_ADDRESS) //The device ID is correct, The correct device ID depends on the AD pin //����ID��ȷ, ����ID����ȷȡ����AD����
    {
        I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_1,0X01);	//Set CLKSEL,PLL X axis as reference //����CLKSEL,PLL X��Ϊ�ο�
        I2C_WriteOneByte(devAddr,MPU6050_RA_PWR_MGMT_2,0X00);	//Acceleration and gyroscope both work //���ٶ��������Ƕ�����
        MPU6050_Set_Rate(200);	                      //Set the sampling rate to 200Hz for FAST-LIO //���ò�����Ϊ50Hz
    } else return 1;
    return 0;
}


/**************************************************************************
Function: Burst read all MPU6050 data (accel + gyro) in one I2C transaction
          替代原来的12次逐字节读取，大幅提升读取效率
Input   : none
Output  : none (updates imu.accel and imu.gyro directly)
**************************************************************************/
void MPU6050_Read_All(void)
{
    uint8_t buf[14];

    // Burst read 14 bytes from ACCEL_XOUT_H (0x3B) to GYRO_ZOUT_L (0x48)
    // MPU6050 auto-increments register address in burst mode
    I2C_ReadBuff(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buf);

    // Parse accelerometer data (big-endian)
    imu.accel.x = ((int16_t)buf[0] << 8) | buf[1];
    imu.accel.y = ((int16_t)buf[2] << 8) | buf[3];
    imu.accel.z = ((int16_t)buf[4] << 8) | buf[5];
    // buf[6-7] = temperature (skipped)
    // Parse gyroscope data (big-endian)
    imu.gyro.x = ((int16_t)buf[8] << 8) | buf[9];
    imu.gyro.y = ((int16_t)buf[10] << 8) | buf[11];
    imu.gyro.z = ((int16_t)buf[12] << 8) | buf[13];

    // Apply zero-drift compensation (same logic as original separate functions)
    if(SysVal.Time_count >= CONTROL_DELAY)
    {
        imu.Original_accel.x = imu.accel.x;
        imu.Original_accel.y = imu.accel.y;
        imu.Original_accel.z = imu.accel.z;
        imu.accel.x = imu.Original_accel.x - imu.Deviation_accel.x;
        imu.accel.y = imu.Original_accel.y - imu.Deviation_accel.y;
        imu.accel.z = imu.Original_accel.z - imu.Deviation_accel.z + 16384;

        imu.Original_gyro.x = imu.gyro.x;
        imu.Original_gyro.y = imu.gyro.y;
        imu.Original_gyro.z = imu.gyro.z;
        imu.gyro.x = imu.Original_gyro.x - imu.Deviation_gyro.x;
        imu.gyro.y = imu.Original_gyro.y - imu.Deviation_gyro.y;
        imu.gyro.z = imu.Original_gyro.z - imu.Deviation_gyro.z;
    }
}

//------------------End of File----------------------------
