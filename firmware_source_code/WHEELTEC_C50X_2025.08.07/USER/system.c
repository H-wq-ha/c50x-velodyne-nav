/***********************************************
Company: WHEELTEC Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V3.5
Update: 2021-01-29

All rights reserved
***********************************************/

#include "system.h"

SYS_VAL_t SysVal;

void systemInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(168);

    /* Detect board hardware by IMU model. */
    I2C_GPIOInit();
    SYS_VAL_t_Init(&SysVal);

    /* USART1 can be used for ROS/upper-computer communication. */
    UART1_Init(460800);

    if (MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID())
    {
        SysVal.HardWare_Ver = V1_0;

        V1_0_LED_Init();
        V1_0_CAN1_Mode_Init(1, 3, 3, 6, 0);
        V1_0_MiniBalance_PWM_Init(16799, 0);

        PS2_Init();
        PS2_Key_Param_Init();
        MPU6050_initialize();
    }
    else if (REG_VAL_WIA == ICM20948_getDeviceID())
    {
        SysVal.HardWare_Ver = V1_1;

        RGB_LightStrip_Init();
        CAN1_Mode_Init(1, 3, 3, 6, 0);
        MiniBalance_PWM_Init(16799, 0);

        invMSInit();
        MX_USB_HOST_Init();
    }
    else
    {
        NVIC_SystemReset();
    }

    Buzzer_Init();
    EnableKey_Init();
    KEY_Init();
    OLED_Init();

#if VECT_TAB_OFFSET == 0x10000
    UART4_Init(230400);
#elif VECT_TAB_OFFSET == 0
    UART4_Init(9600);
#endif

    /* USART3 is the default ROS/upper-computer port. */
    UART3_Init(460800);

    Remoter_Init();

    EncoderA_Init();
    EncoderB_Init();

    ADC1_Init();

#if defined AKM_CAR
    ADC2_Init();
#else
    EncoderC_Init();
    EncoderD_Init();
#endif

    Robot_Select();
    ROBOT_CONTROL_t_Init(&robot_control);

    PI_Controller_Init(&PI_MotorA, robot.V_KP, robot.V_KI);
    PI_Controller_Init(&PI_MotorB, robot.V_KP, robot.V_KI);
    PI_Controller_Init(&PI_MotorC, robot.V_KP, robot.V_KI);
    PI_Controller_Init(&PI_MotorD, robot.V_KP, robot.V_KI);

    auto_recharge_reset();
    OLED_Param_Init(&oled);
    APPKey_Param_Init(&appkey);
    Remoter_Param_Init(&remoter);

#if defined AKM_CAR
    Akm_ServoParam_Init(&Akm_Servo);
#endif

    FlashParam_Read();

#if defined AKM_CAR
    Servo_Senior_Init(10000 - 1, 168 - 1, Akm_Servo.Mid);
    robot.SERVO.Output = Akm_Servo.Mid;

    if (robot.type >= 2 && robot.type != 9)
    {
        short TmpPWM;

        delay_ms(200);
        TmpPWM = get_ServoPWM(get_DMA_SlideRes());
        Servo_Top_Init(10000 - 1, 84 - 1, TmpPWM);
        PI_Controller_Init(&PI_Servo, 0, 0);
        PI_Servo.Output = TmpPWM;
        robot_control.smooth_Servo = TmpPWM;
        robot_control.ServoLow_flag = 1;
    }
#endif

    Buzzer_AddTask(1, 100);
}
