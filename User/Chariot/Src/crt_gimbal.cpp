/**
 * @file crt_gimbal.cpp
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_gimbal.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 云台初始化
 *
 */
void Class_Gimbal::Init()
{
    // imu初始化
    Boardc_BMI.Init();

    // yaw轴电机
    Motor_Yaw.PID_Angle.Init(30.f, 0.0f, 0.0f, 0.0f, 500, 500);
    Motor_Yaw.PID_Omega.Init(60.0f, 15.0f, 0.0f, 0.0f, 6000, Motor_Yaw.Get_Output_Max(), 10.f, 50.f);
    Motor_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Yaw.Get_Output_Max(), Motor_Yaw.Get_Output_Max());
    Motor_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);
<<<<<<< HEAD

    /*test_DM4310*/
    Motor_Yaw_DM4310.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION);
    //CAN_Send_Data(&hfdcan1, DM_Motor_ID_0xA1+0xf0, DM_Motor_CAN_Message_Save_Zero, 8);
=======
>>>>>>> d28e22f2ed8b8045d8d1979d840f7161714beda0
    
    // pitch轴电机
    Motor_Pitch.PID_Angle.Init(22.f, 0.0f, 0.001f, 0.0f, 2.f, 650.f);
    Motor_Pitch.PID_Omega.Init(90.0f, 20.0f, 0.0f, 0.0f, 6000, Motor_Pitch.Get_Output_Max(),0.f,0.f,40.f);
    Motor_Pitch.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_Pitch.Get_Output_Max(), Motor_Pitch.Get_Output_Max());
    Motor_Pitch.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);
}


/**
 * @brief 输出到电机
 *
 */
<<<<<<< HEAD
float Tmp_Target_Yaw_Angle = 0.0f,Tmp_Ture_Yaw_Angle = 0.0f;
=======

>>>>>>> d28e22f2ed8b8045d8d1979d840f7161714beda0
void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);


        Motor_Yaw.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Yaw.PID_Torque.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Pitch.PID_Torque.Set_Integral_Error(0.0f);

        Motor_Yaw.Set_Target_Torque(0.0f);
        Motor_Pitch.Set_Target_Torque(0.0f);

        Motor_Yaw.Set_Out(0.0f);
        Motor_Pitch.Set_Out(0.0f);

    }
    else // 非失能模式
    {
        if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
        {
            //控制方式
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
<<<<<<< HEAD
            Motor_Yaw_DM4310.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
=======
>>>>>>> d28e22f2ed8b8045d8d1979d840f7161714beda0

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

<<<<<<< HEAD
            //处理yaw轴180度问题
            Tmp_Target_Yaw_Angle = Target_Yaw_Angle * PI / 180.0f;
            Tmp_Ture_Yaw_Angle = Motor_Yaw_DM4310.Get_Now_Angle();
            while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)>Max_Yaw_Angle_Radian)
            {
            Tmp_Target_Yaw_Angle -= (2 * Max_Yaw_Angle_Radian);
            }
            while((Tmp_Target_Yaw_Angle-Tmp_Ture_Yaw_Angle)<Min_Yaw_Angle_Radian)
            {
            Tmp_Target_Yaw_Angle += (2 * Max_Yaw_Angle_Radian);
            }

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            /*test_dm4310*/
            Motor_Yaw_DM4310.Set_Target_Angle(Tmp_Target_Yaw_Angle);
            Motor_Yaw_DM4310.Set_Target_Omega(0.0f);
            Motor_Yaw_DM4310.Set_Target_Torque(0.0f);
            Motor_Yaw_DM4310.Set_MIT_K_P(6.0f);
            Motor_Yaw_DM4310.Set_MIT_K_D(0.2f);
=======
            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);
>>>>>>> d28e22f2ed8b8045d8d1979d840f7161714beda0
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
        {
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            Motor_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());
            Motor_Pitch.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle());
           
        }
        else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
        {

            Motor_Pitch.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Yaw.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            // 限制角度
            Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
            Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

            // 设置目标角度
            Motor_Yaw.Set_Target_Angle(Target_Yaw_Angle);
            Motor_Pitch.Set_Target_Angle(Target_Pitch_Angle);

        }
    }
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Gimbal::TIM_Calculate_PeriodElapsedCallback()
{
    //控制模式
    Output();

    //PID输出
    Motor_Yaw.TIM_PID_PeriodElapsedCallback();
<<<<<<< HEAD

    //滑模控制
    // static uint8_t mod10 = 0;
    // if(mod10 == 2){
    //     //注意电机正转角度应该增大，IMU坐标系应该和该坐标系一致，不然会负反馈
    //     Motor_Yaw.Set_Transform_Angle(-Boardc_BMI.Get_Angle_Yaw());
    //     Motor_Yaw.Set_Transform_Omega(-Boardc_BMI.Get_Gyro_Yaw() * 57.3f);          //陀螺仪这里的角度得是度每秒
    //     Motor_Yaw.TIM_SMC_PeriodElapsedCallback();
    //     mod10 = 0;
    // }
    // mod10 ++;

    Motor_Pitch.TIM_PID_PeriodElapsedCallback();

    Motor_Yaw_DM4310.TIM_Process_PeriodElapsedCallback();
=======
    Motor_Pitch.TIM_PID_PeriodElapsedCallback();
>>>>>>> d28e22f2ed8b8045d8d1979d840f7161714beda0
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
