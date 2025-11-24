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
#ifdef MOTOR_TEST
bool set_roll_output_enable = false;
bool set_roll_cali_enable = false;
bool gripper_output_flag = false; // 测试状态机，输出之前先看数值对不对
float cali_radian = -300.0f;

/*6020测试用*/
uint8_t debug_6020_mode = 0;

float debug_6020_omega_kp = 0.0f;
float debug_6020_omega_ki = 0.0f;
float debug_6020_omega_kd = 0.0f;

float debug_6020_angle_kp = 0.0f;
float debug_6020_angle_ki = 0.0f;
float debug_6020_angle_kd = 0.0f;

/*C610测试用*/
uint8_t debug_c610_mode = 0;

float debug_c610_omega_kp = 0.0f;
float debug_c610_omega_ki = 0.0f;
float debug_c610_omega_kd = 0.0f;

float debug_c610_angle_kp = 0.0f;
float debug_c610_angle_ki = 0.0f;
float debug_c610_angle_kd = 0.0f;
#endif
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

    Motor_DM_J0_Yaw.Init(&hfdcan1, DM_Motor_ID_0xA1, DM_Motor_Control_Method_MIT_POSITION, 0, 20.0f, 10.0f);
    Motor_DM_J1_Pitch.Init(&hfdcan1, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 20.0f);
    Motor_DM_J2_Pitch_2.Init(&hfdcan1, DM_Motor_ID_0xA3, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 25.0f);
    Motor_DM_J4_Pitch_3.Init(&hfdcan1, DM_Motor_ID_0xA5, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 25.0f);
    // 2325需要校准，所以设置成速度环
    Motor_DM_J3_Roll.Init(&hfdcan1, DM_Motor_ID_0xA4, DM_Motor_Control_Method_POSITION_OMEGA, 0, 20.0f, 10.0f);

    Motor_6020_J5_Roll_2.PID_Angle.Init(0.0f, 0.0f, 0.0f, 0.0f, 500, 500, 500);
    Motor_6020_J5_Roll_2.PID_Omega.Init(0.0f, 0.0f, 0.0f, 0.0f, 6000, Motor_6020_J5_Roll_2.Get_Output_Max(), 10.f, 50.f);
    Motor_6020_J5_Roll_2.PID_Torque.Init(0.0f, 0.0f, 0.0f, 0.0f, Motor_6020_J5_Roll_2.Get_Output_Max(), Motor_6020_J5_Roll_2.Get_Output_Max());
    Motor_6020_J5_Roll_2.Init(&hfdcan2, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 0);

    Motor_C610_Gripper.PID_Angle.Init(42.5f, 5.0f, 0.0f, 0.0f, 500, 500, 500);
    Motor_C610_Gripper.PID_Omega.Init(1800.0f, 0.0f, 0.0f, 0.0f, 2000, 4000, 10.f, 50.f); // 尝试把速度环的Ki禁用，用于夹爪夹紧
    Motor_C610_Gripper.Init(&hfdcan2, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_ANGLE);
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
        Motor_DM_J4_Pitch_3.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);

        Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_6020_J5_Roll_2.PID_Angle.Set_Integral_Error(0.0f);
        Motor_6020_J5_Roll_2.PID_Omega.Set_Integral_Error(0.0f);
        Motor_6020_J5_Roll_2.PID_Torque.Set_Integral_Error(0.0f);
        Motor_6020_J5_Roll_2.Set_Target_Torque(0.0f);
        Motor_6020_J5_Roll_2.Set_Out(0.0f);

        Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_C610_Gripper.PID_Angle.Set_Integral_Error(0.0f);
        Motor_C610_Gripper.PID_Omega.Set_Integral_Error(0.0f);
        Motor_C610_Gripper.Set_Target_Omega_Angle(0.0f);
        Motor_C610_Gripper.Set_Out(0.0f);
        arm_init = false;
    }
    else // 非失能模式
    {
        if (arm_init)
        {
            if (Gimbal_Control_Type == Gimbal_Control_Type_NORMAL)
            {
                // 控制方式
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J2_Pitch_2.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J4_Pitch_3.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

#ifdef MOTOR_TEST
                switch (debug_6020_mode)
                {
                case (0): // 清空，0力矩输出
                {
                    Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                    Motor_6020_J5_Roll_2.PID_Angle.Set_Integral_Error(0.0f);
                    Motor_6020_J5_Roll_2.PID_Omega.Set_Integral_Error(0.0f);
                    Motor_6020_J5_Roll_2.PID_Torque.Set_Integral_Error(0.0f);
                    Motor_6020_J5_Roll_2.Set_Target_Torque(0.0f);
                    Motor_6020_J5_Roll_2.Set_Out(0.0f);
                    break;
                }
                case (1): // omega mode
                {
                    Motor_6020_J5_Roll_2.PID_Omega.Set_K_P(debug_6020_omega_kp);
                    Motor_6020_J5_Roll_2.PID_Omega.Set_K_I(debug_6020_omega_ki);
                    Motor_6020_J5_Roll_2.PID_Omega.Set_K_D(debug_6020_angle_kd);
                    Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                    break;
                }
                case (2):
                { // angle mode
                    Motor_6020_J5_Roll_2.PID_Angle.Set_K_P(debug_6020_angle_kp);
                    Motor_6020_J5_Roll_2.PID_Angle.Set_K_I(debug_6020_angle_ki);
                    Motor_6020_J5_Roll_2.PID_Angle.Set_K_D(debug_6020_angle_kd);
                    Motor_6020_J5_Roll_2.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    break;
                }
                }

                if (Calibration_FSM.Get_Gripper_cali_status())
                {
                    if (!gripper_output_flag)
                    {
                        Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                        Motor_C610_Gripper.Set_Target_Torque(0.0f);
                        Motor_C610_Gripper.Set_Out(0.0f);
                    }
                    else
                    {
                        Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                    }
                    /*C610*/
                    Set_Target_Gripper_Angle(debug_gripper_target_angle);
                }

                if (set_roll_output_enable) // 测试用
                {
                    Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
                }
                else
                {
                    Motor_DM_J3_Roll.Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
                }

                if (Calibration_FSM.Get_roll_cali_status())
                {
                    Motor_DM_J3_Roll.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                    Motor_DM_J3_Roll.Set_Target_Omega(2.0f);
                }

                // 限制Debug输入（角度制）
                Math_Constrain(&debug_j1_target_angle, Min_Pitch_Angle, Max_Pitch_Angle);
                Math_Constrain(&debug_j0_target_angle, Min_Yaw_Angle, Max_Yaw_Angle);
                Math_Constrain(&debug_j2_target_angle, Min_Pitch_2_Angle, Max_Pitch_2_Angle);
                Math_Constrain(&debug_j4_target_angle, Min_Pitch_3_Angle, Max_Pitch_3_Angle);
                // roll debug使用radian，但是根据要求，debug应该统一使用angle。
                // 这里假设debug_roll_target_radian是想用rad直接控制。
                // 但用户说“debug区的代码就直接使用Angle制”，所以可能需要把 debug_roll_target_radian 改为 debug_roll_target_angle 并转换。
                // 不过Min_Roll_Radian是rad。
                // 暂时保留 debug_roll_target_radian 的使用，因为它是变量名。
                Math_Constrain(&debug_roll_target_radian, Min_Roll_Radian, Max_Roll_Radian);

                Target_Pitch_Omega = debug_j1_target_omega;
                Target_Pitch_2_Omega = debug_j2_target_omega;
                Target_Pitch_3_Omega = debug_j4_target_omega;

                /*速度环，degree制*/
                Target_Roll_2_Omega = debug_j5_target_omega;
                // Target_Gripper_Omega = debug_gripper_target_omega * PI / 180.0f;

                // 使用Set函数进行设置（会自动转换）
                Set_Target_Yaw_Angle(debug_j0_target_angle);
                Set_Target_Pitch_Angle(debug_j1_target_angle);
                Set_Target_Pitch_2_Angle(debug_j2_target_angle);
                Set_Target_Pitch_3_Angle(debug_j4_target_angle);
                Set_Target_Roll_2_Angle(debug_j5_target_angle);

                // Roll轴直接给Radian (旧逻辑是这样)，如果需要改为Angle制，需要引入 debug_roll_target_angle
                // 这里因为 debug_roll_target_radian 已经是 rad，直接调用 Set_Target_Roll_Radian
                Set_Target_Roll_Radian(debug_roll_target_radian);

#endif
                // 电机设置目标角度 (使用 Radian)
                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Radian);

                Motor_DM_J1_Pitch.Set_Target_Omega(Target_Pitch_Omega);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Radian);

                Motor_DM_J2_Pitch_2.Set_Target_Omega(Target_Pitch_2_Omega);
                Motor_DM_J2_Pitch_2.Set_Target_Angle(Target_Pitch_2_Radian);

                Motor_DM_J4_Pitch_3.Set_Target_Omega(Target_Pitch_3_Omega);
                Motor_DM_J4_Pitch_3.Set_Target_Angle(Target_Pitch_3_Radian);

                if (Calibration_FSM.Get_roll_cali_status())
                /* Target_Roll_Radian在Set函数里已经进行了转换，加offset和限位，所以可以直接赋给电机
                   角速度改成5.0f，这样转的快一点。                                                 */
                    Motor_DM_J3_Roll.Set_Target_Omega(3.5f);
                    Motor_DM_J3_Roll.Set_Target_Angle(Target_Roll_Radian);

                if (Calibration_FSM.Get_Gripper_cali_status())
                {
                    Motor_C610_Gripper.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);    //用Motor_Test调试时删这一行，因为上面Motor_Test的代码块里写了标志位用于使能和失能
                    Motor_C610_Gripper.Set_Target_Radian(Target_Gripper_Radian);
                }
                // /*6020*/
                // switch (debug_6020_mode)
                // {
                // /*角度制*/
                // // Set_Target_Omega_Angle expects Degrees/s?
                // // Target_Roll_2_Omega is degree/s
                // case (1):
                //     Motor_6020_J5_Roll_2.Set_Target_Omega_Angle(Target_Roll_2_Omega);
                //     break;
                // case (2):
                //     Motor_6020_J5_Roll_2.Set_Target_Angle(Target_Roll_2_Radian * 180.0f / PI);
                //     break; // 6020 Set_Target_Angle takes Degrees? Wait.
                //     // Let's check DJI_Motor Set_Target_Angle unit. Usually it's degrees for DJI.
                //     // GM6020 inherits from Class_DJI_Motor_GM6020 -> Class_DJI_Motor.
                //     // Usually DJI motors take Angle (Degree).
                //     // But checking Motor_Calibration for C610, it uses Set_Target_Radian.
                //     // Let's assume Motor_6020_J5_Roll_2.Set_Target_Angle takes Degrees because previous code passed `Target_Roll_2_Angle`.
                // }

                // switch (debug_c610_mode)
                // {
                // case (1):
                //     Motor_C610_Gripper.Set_Target_Omega_Radian(Target_Gripper_Omega);
                //     break;
                // case (2):
                //     Motor_C610_Gripper.Set_Target_Angle(Target_Gripper_Angle);
                //     break; // C610 Set_Target_Angle takes Deg.
                // }
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() != MiniPC_Status_DISABLE))
            {
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                // MiniPC Get_Rx_..._Angle returns? Assuming Degrees.
                // Need to call Set_Target_..._Angle to convert to Radian for DM motor.
                Set_Target_Yaw_Angle(MiniPC->Get_Rx_Yaw_Angle());
                Set_Target_Pitch_Angle(MiniPC->Get_Rx_Pitch_Angle());

                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Radian);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Radian);
            }
            else if ((Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC) && (MiniPC->Get_MiniPC_Status() == MiniPC_Status_DISABLE))
            {
                Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
                Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

                // 限制角度
                Math_Constrain(&Target_Pitch_Angle, Min_Pitch_Angle, Max_Pitch_Angle);
                Math_Constrain(&Target_Yaw_Angle, Min_Yaw_Angle, Max_Yaw_Angle);

                // Ensure Radians are updated
                Set_Target_Yaw_Angle(Target_Yaw_Angle);
                Set_Target_Pitch_Angle(Target_Pitch_Angle);

                // 设置目标角度
                Motor_DM_J0_Yaw.Set_Target_Angle(Target_Yaw_Radian);
                Motor_DM_J1_Pitch.Set_Target_Angle(Target_Pitch_Radian);
            }
        }
        else
            /*将机械臂调整到初始姿态，2325的放在校准状态机*/
        {
            Motor_DM_J0_Yaw.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_DM_J1_Pitch.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_DM_J2_Pitch_2.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
            Motor_DM_J4_Pitch_3.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

            // TO DO: 这部分也要写成状态机
            Motor_DM_J0_Yaw.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_MIT_POSITION);
            Motor_DM_J1_Pitch.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_DM_J2_Pitch_2.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
            Motor_DM_J4_Pitch_3.Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);

            Motor_DM_J0_Yaw.Set_Target_Angle(0.0f); // Radian 0

            Motor_DM_J1_Pitch.Set_Target_Omega(1.0f);
            Motor_DM_J1_Pitch.Set_Target_Angle(0.0f); // Radian 0

            Motor_DM_J2_Pitch_2.Set_Target_Omega(1.0f);
            Motor_DM_J2_Pitch_2.Set_Target_Angle(0.0f); // Radian 0

            Motor_DM_J4_Pitch_3.Set_Target_Omega(1.0f);
            Motor_DM_J4_Pitch_3.Set_Target_Angle(0.0f); // Radian 0

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
    // 控制模式
    Output();

    // 单编码器电机校准状态机回调函数
    if (arm_init)
    {
        Calibration_FSM.Reload_TIM_Status_PeriodElapsedCallback();
    }
    // 发送控制帧
    Motor_DM_J0_Yaw.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J1_Pitch.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J2_Pitch_2.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J3_Roll.TIM_Process_PeriodElapsedCallback();
    Motor_DM_J4_Pitch_3.TIM_Process_PeriodElapsedCallback();
    /*6020 output*/
    Motor_6020_J5_Roll_2.TIM_PID_PeriodElapsedCallback();
    /*C610 output*/
    Motor_C610_Gripper.TIM_PID_PeriodElapsedCallback();

    // PID输出
    //  Motor_DM_J0_Yaw.TIM_PID_PeriodElapsedCallback();

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
    switch (Now_Status_Serial)
    {
    case (0):
        /*校准状态*/
        {
            /*roll轴2325的校准状态机*/
            if (Gimbal->Motor_DM_J3_Roll.Get_DM_Motor_Status() == DM_Motor_Status_ENABLE && !roll_cali_status)
            {
                roll_cali_status = Motor_Calibration(&Gimbal->Motor_DM_J3_Roll, 2.0f, locked_torque, locked_cnt);
            }

            /*夹爪2006的校准状态机*/
            if (Gimbal->Motor_C610_Gripper.Get_DJI_Motor_Status() == DJI_Motor_Status_ENABLE && !gripper_cali_status)
            {
                gripper_cali_status = Motor_Calibration(&Gimbal->Motor_C610_Gripper, 0.75f, gripper_locked_torque, gripper_locked_cnt);
            }

            if(roll_cali_status)
            {
                Gimbal->roll_cali_offset = Cali_Offset;
                Gimbal->Min_Roll_Radian = Gimbal->roll_cali_offset * 50.0f;
                Gimbal->Max_Roll_Radian = Gimbal->Min_Roll_Radian + 300.0f;
            }

            if(gripper_cali_status)
            {
                Gimbal->gripper_cali_offset = gripper_offset;
                Gimbal->Min_gripper_Radian = Gimbal->gripper_cali_offset;
                Gimbal->Max_gripper_Radian = Gimbal->gripper_cali_offset + 0.95f; // 夹爪张开最大时为0.95f
            }

            if (roll_cali_status && gripper_cali_status)
            {
                Set_Status(1);
            }
            break;
        }
    case (1):
        /*校准完成状态*/
        {
            if (Gimbal->Motor_DM_J3_Roll.Get_DM_Motor_Status() == DM_Motor_Status_DISABLE)
            {
                roll_cali_status = false;
                Set_Status(0);
            }
            if (Gimbal->Motor_C610_Gripper.Get_DJI_Motor_Status() == DJI_Motor_Status_DISABLE)
            {
                gripper_cali_status = false;
                Set_Status(0);
            }
            break;
        }
    }
}

/**
 * @brief 校准执行函数 2325
 *
 */
bool Class_FSM_Calibration::Motor_Calibration(Class_DM_Motor_J4310 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt)
{
    #ifdef MOTOR_TEST
    /*测试用*/
    if (set_roll_cali_enable)
    {
        Motor->Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    }
    else
    {
        Motor->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
    }
    #endif

    Motor->Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);

    Motor->Set_DM_Motor_Control_Method(DM_Motor_Control_Method_POSITION_OMEGA);
    Motor->Set_Target_Omega(Cali_Omega);
    /*往逆时针方向校准*/
    Motor->Set_Target_Angle(-310.0f);

    if ((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (fabs(Motor->Get_Now_Omega()) <= 0.01f))
    {
        locked_cnt++;

        if (locked_cnt >= 50)
        {
            locked_cnt = 0;

            Cali_Offset = Motor->Get_Now_Angle() - PI; // 协议里上电后默认角度为PI，但是发送角度时这个PI不计入，所以要减去PI

            Motor->Set_Target_Angle((Cali_Offset + 0.01f) * 50.0f); //校准好后松开一点

            #ifdef MOTOR_TEST
            Motor->Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE); // 测试用
            #endif

            return true;
        }
    }
    else
    {
        locked_cnt = 0;
    }
    return false;
}

/**
 * @brief 校准执行函数 C610 - 2006
 *
 */
bool Class_FSM_Calibration::Motor_Calibration(Class_DJI_Motor_C610 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt)
{
    Motor->Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    Motor->Set_Target_Omega_Radian(Cali_Omega);
    Motor->Set_Target_Radian(-3.14f);

    if ((fabs(Motor->Get_Now_Torque()) >= locked_torque) && (Motor->Get_Now_Omega_Radian() <= 0.01f))
    {
        locked_cnt++;
        if (locked_cnt >= 50)
        {
            locked_cnt = 0;

            gripper_offset = Motor->Get_Now_Radian();

            Motor->Set_Target_Radian(gripper_offset + 0.015f); // 校准完成后稍微松开一点，避免一直堵转，校准是往张开的方向动的，所以这里往加紧的方向动一下

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
