/**
 * @file tsk_config_and_callback.cpp
 * @author lez by yssickjgd
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 * @copyright ZLLC 2024
 */

/**

 *
 */

/**
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"
#include "drv_tim.h"
#include "drv_dwt.h"
#include "config.h"
#include "drv_can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h" // ::CMSIS:RTOS2
#include "task.h"
#include "crt_image.h"
#include "Steering_Wheel_Task.h"
#include "dvc_dmimu.h"
/* Private macros ------------------------------------------------------------*/
//osThreadId_t steering_wheel_taskHandle;
//const osThreadAttr_t steering_wheel_task_attributes = {
//    .name = "steering_wheel_task",
//    .stack_size = 528 * 4,
//    .priority = (osPriority_t)osPriorityRealtime,
//};
/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


/* Private function declarations ---------------------------------------------*/
Class_Image Image;
Class_DM_IMU IMU;
/* Function prototypes -------------------------------------------------------*/
#ifdef STEERING_WHEEL
/**
 * @brief
 *
 */
void Agv_Board_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x201):
    {
        steering_wheel.Directive_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;

    case (0x202):
    {
        steering_wheel.Motion_Motor.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case ENCODER_ID:
    {
        steering_wheel.Encoder.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    }
}

/**
 * @brief
 *
 */
int test_steering_wheel_start;
int test_steering_wheel_dt;
int test_cnt;
float test_fps;
void Agv_Board_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (AGV_BOARD_ID):
    case 0x01E:
    {
				test_fps = FPS_Counter_Update();
        steering_wheel.CAN_RxChassisCallback(CAN_RxMessage);
    }
    break;
    case (0x02A):
    case (0x02B):
    case (0x02C):
    case (0x02D):
    case (0x03A):
    case (0x03B):
    case (0x03C):
    case (0x03D):
    {
        steering_wheel.CAN_RxAgvBoardCallback(CAN_RxMessage);
    }
    break;
    default:
         break;
    }

}

#endif

void Image_CAN1_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x205):
    {
        Image.Motor_Image_Pitch.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x206):
    {
        Image.Motor_Image_Roll.CAN_RxCpltCallback(CAN_RxMessage->Data);
    }
    break;
    case (0x11):
    {
        IMU.IMU_UpdateData(CAN_RxMessage->Data);
    }
    break;
	}
}
void Image_CAN2_Callback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    switch (CAN_RxMessage->Header.StdId)
    {
    case (0x02E):
    {
        float rx_pitch_angle, rx_roll_angle;
        memcpy(&rx_pitch_angle, CAN_RxMessage->Data, sizeof(float));
        memcpy(&rx_roll_angle, CAN_RxMessage->Data + 4, sizeof(float));
        Image.Set_Target_Image_Pitch_Angle(rx_pitch_angle);
        Image.Set_Target_Image_Roll_Angle(rx_roll_angle);
    }
    break;
    }
}
/**
 * @brief 初始化任务
 *
 */
extern "C" void
Task_Init()
{

    DWT_Init(72);

    /********************************** 驱动层初始化 **********************************/
#ifdef STEERING_WHEEL
    /**
     * @brief
     *
     */
	
    CAN_Init(&hcan1, Image_CAN1_Callback);
    CAN_Init(&hcan2, Image_CAN2_Callback);

#endif // DEBUG

    /********************************* 设备层初始化 *********************************/
    // 设备层集成在交互层初始化中，没有显视地初始化

    /********************************* 交互层初始化 *********************************/
    //steering_wheel.Init();    
    Image.Init();

	//OSTaskInit();
}

/**
 * @brief 前台循环任务
 *
 */
uint8_t can_tx_status = 1;
extern "C" void Task_Loop()
{
    static uint16_t mod150 = 0;
	  mod150++;
    if(mod150 == 150)
    {
        Image.Motor_Image_Pitch.TIM_Alive_PeriodElapsedCallback();
        Image.Motor_Image_Roll.TIM_Alive_PeriodElapsedCallback();
        
        mod150 = 0;
    }
    Image.TIM_Calculate_PeriodElapsedCallback();
    static uint8_t mod5 = 0;
    mod5++;
    if(mod5 == 5)
    {
        CAN_Send_Data(&hcan1,0x1FF,CAN1_0x1ff_Tx_Data,8);
        mod5 = 0;
    }
    else
    {
        can_tx_status = IMU.IMU_RequestData(&hcan1,0x01,3);
    }

    memcpy(&CAN1_0x0A_Tx_Data,&IMU.Data.yaw,4);
    //CAN_Send_Data(&hcan1,0x0A,CAN1_0x0A_Tx_Data,8);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
