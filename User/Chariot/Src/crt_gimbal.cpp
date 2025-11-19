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
bool set_roll_output_enable = false;
bool set_roll_cali_enable = false;
float cali_radian = -300.0f;
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

    Motor_DM_J0_Yaw.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION, 0, 20.0f,10.0f);
    Motor_DM_J1_Pitch.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 20.0f);
    Motor_DM_J2_Pitch_2.Init(&hfdcan1, DM_Motor_ID_0xA3, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 25.0f);
    //2325需要校准，所以设置成速度环
    Motor_DM_J3_Roll.Init(&hfdcan1, DM_Motor_ID_0xA4, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);

    /*初始化状态机，不进行初始化的话状态机没法控制电机什么的*/
    Calibration_FSM.Gimbal = this;

    /*给MIT模式的电机设置的MIT参数*/
    Motor_DM_J0_Yaw.Set_MIT_K_P(5.0f);
    Motor_DM_J0_Yaw.Set_MIT_K_D(2.0f);

    // // yaw轴电机
    // Motor_DM_J0_Yaw.PID_Angle.Init(30.f, 0.0f, 0.0f, 0.0f, 500, 500);
    // Motor_DM_J0_Yaw.PID_Omega.Init(60.0f, 15.0f, 0.0f, 0.0f, 6000, Motor_DM_J0_Yaw.Get_Output_Max(), 10.f, 50.f);
    // Motor_DM_J0_Yaw.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_DM_J0_Yaw.Get_Output_Max(), Motor_DM_J0_Yaw.Get_Output_Max());
    // Motor_DM_J0_Yaw.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2048);
    
    // // pitch轴电机
    // Motor_DM_J1_Pitch.PID_Angle.Init(22.f, 0.0f, 0.001f, 0.0f, 2.f, 650.f);
    // Motor_DM_J1_Pitch.PID_Omega.Init(90.0f, 20.0f, 0.0f, 0.0f, 6000, Motor_DM_J1_Pitch.Get_Output_Max(),0.f,0.f,40.f);
    // Motor_DM_J1_Pitch.PID_Torque.Init(0.f, 0.0f, 0.0f, 0.0f, Motor_DM_J1_Pitch.Get_Output_Max(), Motor_DM_J1_Pitch.Get_Output_Max());
    // Motor_DM_J1_Pitch.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE, 3413);
}

/**
 * @brief 输出到电机
 *
 */

void Class_Gimbal::Output()
{
    if (Gimbal_Control_Type == Gimbal_Control_Type_DISABLE)
    {
        // 云台失能
        Motor_DM_J0_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J1_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J2_Pitch_2.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        arm_init = false;
    }
    else // 非失能模式
    {
        if(arm_init)
        {
            if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
            {
                // 控制方式
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J2_Pitch_2.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                if(set_roll_output_enable)  //测试用
                {
                    Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
                }
                else
                {
                    Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                }


                if(Calibration_FSM.Get_roll_cali_status())
                {
                    Motor_DM_J3_Roll.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                    Motor_DM_J3_Roll.Set_Target_Omega(2.0f);
                }
                #ifdef DEBUG
                Math_Constrain(&debug_j1_target_angle, Min_Pitch_Angle, Max_Pitch_Angle); // 对测试用的角度进行限幅
                Math_Constrain(&debug_j0_target_angle, Min_Yaw_Angle, Max_Yaw_Angle);     // 同上
                Math_Constrain(&debug_j2_target_angle, Min_Pitch_2_Angle, Max_Pitch_2_Angle);
                Math_Constrain(&debug_roll_target_radian, Min_Roll_Radian, Max_Roll_Radian);

                // debug_j0_target_radian = debug_j0_target_angle * 3.14 / 180;
                // debug_j1_target_radian = debug_j1_target_angle * 3.14 / 180;
                // debug_j2_target_radian = debug_j2_target_angle * 3.14 / 180;

                Target_Yaw_Angle = debug_j0_target_radian;                                // 协议中使用弧度控制，这里把测试用的弧度赋给Target_Angle
                Target_Pitch_Angle = debug_j1_target_radian;                              // 同上
                Target_Pitch_2_Angle = debug_j2_target_radian;

                Target_Pitch_Omega = debug_j1_target_omega;
                Target_Pitch_2_Omega = debug_j2_target_omega;

                Target_Roll_Angle = debug_roll_target_radian;

                #endif

                // 为电机设置目标角度

                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Angle);

                Motor_DM_J1_Pitch.Set_Target_Omega(Target_Pitch_Omega);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Angle);

                Motor_DM_J2_Pitch_2.Set_Target_Omega(Target_Pitch_2_Omega);
                Motor_DM_J2_Pitch_2.Set_Target_Angle(Target_Pitch_2_Angle);
                if(Calibration_FSM.Get_roll_cali_status())
                    Motor_DM_J3_Roll.Set_Target_Angle(Target_Roll_Angle + roll_cali_offset * 50.0f);
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
            {
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                Motor_DM_J0_Yaw.Set_Target_Angle(MiniPC->Get_Rx_Yaw_Angle());
                Motor_DM_J1_Pitch.Set_Target_Angle(MiniPC->Get_Rx_Pitch_Angle());
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
            {

                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                // 限制角度
                Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
                Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

                // 设置目标角度
                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Angle);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Angle);
            }
        }
        else
        {
            /*将机械臂调整到初始姿态，2325的先不写*/
            Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
            Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_DM_J2_Pitch_2.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

            Motor_DM_J0_Yaw.Set_Target_Angle(0.0f);

            Motor_DM_J1_Pitch.Set_Target_Omega(0.5f);
            Motor_DM_J1_Pitch.Set_Target_Angle(0.0f);

            Motor_DM_J2_Pitch_2.Set_Target_Omega(0.5f);
            Motor_DM_J2_Pitch_2.Set_Target_Angle(0.0f);

            arm_init = true;
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

    //单编码器电机校准状态机回调函数
    if(arm_init)
    {
    Calibration_FSM.Reload_TIM_Status_PeriodElapsedCallback();
    }
    //发送控制帧
    Motor_DM_J0_Yaw.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J1_Pitch.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J2_Pitch_2.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J3_Roll.TIM_Process_PeriodElapsedCallback();

    //PID输出
    // Motor_DM_J0_Yaw.TIM_PID_PeriodElapsedCallback();

    // //滑模控制
    // // static uint8_t mod10 = 0;
    // // if(mod10 == 2){
    // //     //注意电机正转角度应该增大，IMU坐标系应该和该坐标系一致，不然会负反馈
    // //     Motor_DM_J0_Yaw.Set_Transform_Angle(-Boardc_BMI.Get_Angle_Yaw());
    // //     Motor_DM_J0_Yaw.Set_Transform_Omega(-Boardc_BMI.Get_Gyro_Yaw() * 57.3f);          //陀螺仪这里的角度得是度每秒
    // //     Motor_DM_J0_Yaw.TIM_SMC_PeriodElapsedCallback();
    // //     mod10 = 0;
    // // }
    // // mod10 ++;

    // Motor_DM_J1_Pitch.TIM_PID_PeriodElapsedCallback();
}

/**
 * @brief 单编码器电机校准状态机
 *
 */
void Class_FSM_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;
    switch(Now_Status_Serial)
    {
        case(0):
        /*校准状态*/
        {
            /*roll轴2325的校准状态机*/
            if(Gimbal->Motor_DM_J3_Roll.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !roll_cali_status)
            {
                roll_cali_status = Motor_Calibration(&Gimbal->Motor_DM_J3_Roll, 1.0f, locked_torque, locked_cnt);
            }

            if(roll_cali_status) 
            {
                Gimbal->roll_cali_offset = Cali_Offset;
                Gimbal->Min_Roll_Radian = Gimbal->roll_cali_offset * 50.0f;
                Gimbal->Max_Roll_Radian = Gimbal->Min_Roll_Radian + 300.0f;

                Set_Status(1);
            }
            break;
        }
        case(1):
        /*校准完成状态*/
        {
            if(Gimbal->Motor_DM_J3_Roll.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
            {
                roll_cali_status = false;
                Set_Status(0);
            }
            break;
        }
    }
}

/**
 * @brief 校准执行函数
 *
 */
bool Class_FSM_Calibration::Motor_Calibration(Class_DM_Motor_J4310* Motor, float Cali_Omega, float locked_torque, uint16_t& locked_cnt)
{
    /*测试用*/
    if(set_roll_cali_enable)
    {
        Motor->Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    }
    else
    {
        Motor->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }

    Motor->Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
    Motor->Set_Target_Omega(Cali_Omega);
    /*往逆时针方向校准*/
    Motor->Set_Target_Angle(cali_radian);

    if((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (fabs(Motor->Get_Now_Omega()) <= 0.01f))
    {
        locked_cnt++;

        if(locked_cnt >= 50)
        {
            locked_cnt = 0;

            Cali_Offset = Motor->Get_Now_Angle() - PI;

            Motor->Set_Target_Angle(Cali_Offset);

            Motor->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);      //测试用

            return true;
        }
    }
    else
    {
        locked_cnt = 0;
    }
    return false;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
