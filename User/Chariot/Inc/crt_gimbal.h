/**
 * @file crt_gimbal.h
 * @author cjw
 * @brief 云台
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

#ifndef CRT_GIMBAL_H
#define CRT_GIMBAL_H

#define GEAR_RATIO 2 // roll轴减速比18:36

/* Includes ------------------------------------------------------------------*/

#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_imu.h"
#include "dvc_lkmotor.h"
#include "dvc_dmmotor.h"
#include "alg_fsm.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 云台控制类型
 *
 */
enum Enum_Gimbal_Control_Type : uint8_t
{
    Gimbal_Control_Type_DISABLE = 0,
    Gimbal_Control_Type_NORMAL,
    Gimbal_Control_Type_MINIPC,
};

enum Enum_Motor_DM_J0_Yaw_Type : uint8_t
{
    Yaw_A = 0,
    Yaw_B,
};

struct IMU_Data
{
    float Pitch;
    float Roll;
    float Yaw;
    float Omega_X;
    float Omega_Y;
    float Omega_Z;
};

/**
 * @brief Specialized, yaw轴电机类
 *
 */
class Class_Gimbal_Yaw_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_Trer_Rad_Yaw();
    inline float Get_True_Gyro_Yaw();
    inline float Get_True_Angle_Yaw();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    // 常量

    // 内部变量
    float True_Rad_Yaw = 0.0f;
    float True_Angle_Yaw = 0.0f;
    float True_Gyro_Yaw = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

float Class_Gimbal_Yaw_Motor_GM6020::Get_Trer_Rad_Yaw()
{
    return (True_Rad_Yaw);
}

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Gyro_Yaw()
{
    return (True_Gyro_Yaw);
}

float Class_Gimbal_Yaw_Motor_GM6020::Get_True_Angle_Yaw()
{
    return (True_Angle_Yaw);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_GM6020 : public Class_DJI_Motor_GM6020
{
public:
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    // 内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_GM6020::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);
}

/**
 * @brief Specialized, pitch轴电机类
 *
 */
class Class_Gimbal_Pitch_Motor_LK6010 : public Class_LK_Motor
{
public:
    // 陀螺仪获取云台角速度
    Class_IMU *IMU;

    inline float Get_True_Rad_Pitch();
    inline float Get_True_Gyro_Pitch();
    inline float Get_True_Angle_Pitch();

    void Transform_Angle();

    void TIM_PID_PeriodElapsedCallback();

protected:
    // 初始化相关变量

    // 常量

    // 重力补偿
    float Gravity_Compensate = 0.0f;

    // 内部变量
    float True_Rad_Pitch = 0.0f;
    float True_Angle_Pitch = 0.0f;
    float True_Gyro_Pitch = 0.0f;
    // 读变量

    // 写变量

    // 读写变量

    // 内部函数
};

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Rad_Pitch()
{
    return (True_Rad_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Angle_Pitch()
{
    return (True_Angle_Pitch);
}

float Class_Gimbal_Pitch_Motor_LK6010::Get_True_Gyro_Pitch()
{
    return (True_Gyro_Pitch);
}

class Class_Gimbal;
class Class_FSM_Calibration : public Class_FSM
{
public:
    Class_Gimbal *Gimbal;
    void Reload_TIM_Status_PeriodElapsedCallback();

    /*电机校准执行函数*/
    bool Motor_Calibration(Class_DM_Motor_J4310 *Motor, float Cali_Omega, float locked_torque, uint16_t &locked_cnt);

    inline bool Get_roll_cali_status();

protected:
    /*roll轴校准相关变量*/
    float Cali_Offset = 0.0f;      // 存储校准后的偏差, rad
    float locked_torque = 1.5f;    // 堵转力矩
    bool roll_cali_status = false; // 校准状态，初始为false
    uint16_t locked_cnt = 0;       // 堵转时间计数
};

/**
 * @brief Specialized, 云台类
 *
 */
class Class_Gimbal
{
public:
    // imu对象
    Class_IMU Boardc_BMI;

    Class_MiniPC *MiniPC;

    Class_DM_Motor_J4310 Motor_DM_J0_Yaw;     // J0 - DM4310
    Class_DM_Motor_J4310 Motor_DM_J1_Pitch;   // J1 - DM8009P
    Class_DM_Motor_J4310 Motor_DM_J2_Pitch_2; // J2 - DM4340
    Class_DM_Motor_J4310 Motor_DM_J3_Roll;    // J3 - DM2325

    Class_FSM_Calibration Calibration_FSM;
    friend class Class_FSM_Calibration;

    /*机械臂初始化标志位*/
    bool arm_init = false;

    /*后期yaw pitch这两个类要换成其父类，大疆电机类*/

    // yaw轴电机
    // Class_DJI_Motor_GM6020 Motor_DM_J0_Yaw;

    // // pitch轴电机
    // Class_DJI_Motor_GM6020 Motor_DM_J1_Pitch;

    void Init();

    inline float Get_Target_Yaw_Angle();
    inline float Get_Target_Pitch_Angle();
    inline float Get_Target_Pitch_2_Angle();
    inline float Get_Target_Roll_Angle();
    inline Enum_Gimbal_Control_Type Get_Gimbal_Control_Type();

    inline void Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type);
    inline void Set_Target_Yaw_Angle(float __Target_Yaw_Angle);
    inline void Set_Target_Pitch_Angle(float __Target_Pitch_Angle);
    inline void Set_Target_Pitch_2_Angle(float __Target_Pitch_2_Angle);
    inline void Set_Target_Roll_Angle(float __Target_Roll_Angle);

    // 由于减速比和多圈的问题存在，所以要写一个专门用于输出轴的角度设定函数
    inline void Set_Target_Roll_Output_Angle(float Target_Angle);

    void TIM_Calculate_PeriodElapsedCallback();

#ifdef DEBUG

    float debug_j0_target_angle = 0.0f; // J0目标角度（角度）
    float debug_j0_target_radian = debug_j0_target_angle * 3.14 / 180;
    float debug_j0_target_omega = 1.0f; // J0目标速度（rad/s）

    float debug_j1_target_angle = 0.0f; // J1目标角度（角度）
    float debug_j1_target_radian = debug_j1_target_angle * 3.14 / 180;
    float debug_j1_target_omega = 1.0f; // J1目标速度（rad/s）

    float debug_j2_target_angle = 0.0f; // J2目标角度（角度）
    float debug_j2_target_radian = debug_j2_target_angle * 3.14 / 180;
    float debug_j2_target_omega = 1.0f; // J2目标速度（rad/s）

    float debug_kp = 1.8f;
    float debug_kd = 1.0f; // 空载下设置为0.2f，否则可能导致电机持续加速
    float debug_roll_target_torque = 0.0f;
    float debug_roll_target_omega = 1.5f;
    float debug_roll_target_radian = 0.0f; // roll目标位置，弧度制，用于在校准后角度的基础上进行增量，顺时针方向为正，电机校准的方向是逆时针，所以需要加角度
#endif
protected:
    // 初始化相关常量
    float Gimbal_Head_Angle;
    // 常量
    float CRUISE_SPEED_YAW = 100.f;
    float CRUISE_SPEED_PITCH = 70.f;

    // yaw轴最小值
    float Min_Yaw_Angle = -180.0f;
    // yaw轴最大值
    float Max_Yaw_Angle = 180.0f;

    // yaw总角度
    float Yaw_Total_Angle;
    float Yaw_Half_Turns;

    // pitch轴最小值
    float Min_Pitch_Angle = 0.0f; // 角度，非弧度
    // pitch轴最大值
    float Max_Pitch_Angle = 120.0f; // 角度，非弧度

    // pitch2轴最小值与最大值
    float Min_Pitch_2_Angle = 0.0f;
    float Max_Pitch_2_Angle = 90.0f;

    // roll校准角度，默认为0
    float roll_cali_offset = 0.0f;
    // roll轴最小值与最大值
    float Min_Roll_Radian = roll_cali_offset * 50.0f;
    float Max_Roll_Radian = Min_Roll_Radian + 300.0f;

    // 内部变量

    // 读变量

    // 写变量

    // 云台状态
    Enum_Gimbal_Control_Type Gimbal_Control_Type = Gimbal_Control_Type_DISABLE;

    // 读写变量

    // yaw轴角度 degree & yaw轴角速度 rad/s
    float Target_Yaw_Angle = 0.0f;
    float Target_Yaw_Omega = 1.0f;

    // pitch轴角度 degree
    float Target_Pitch_Angle = 0.0f;
    float Target_Pitch_Omega = 1.0f;

    // pitch_2角度 degree
    float Target_Pitch_2_Angle = 0.0f;
    float Target_Pitch_2_Omega = 1.0f;

    // roll角度 radian
    float Target_Roll_Angle = 0.0f;
    float Target_Roll_Omega = 1.0f;

    // 内部函数

    void Output();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取yaw轴角度
 *
 * @return float yaw轴角度
 */
float Class_Gimbal::Get_Target_Yaw_Angle()
{
    return (Target_Yaw_Angle);
}
/**
 * @brief 获取pitch轴角度
 *
 * @return float pitch轴角度
 */
float Class_Gimbal::Get_Target_Pitch_Angle()
{
    return (Target_Pitch_Angle);
}
/**
 * @brief 获取pitch_2轴角度
 *
 * @return float pitch_2轴角度
 */
float Class_Gimbal::Get_Target_Pitch_2_Angle()
{
    return (Target_Pitch_2_Angle);
}
/**
 * @brief 获取roll轴角度
 *
 * @return float roll轴角度
 */
float Class_Gimbal::Get_Target_Roll_Angle()
{
    return (Target_Roll_Angle);
}

/**
 * @brief 获取云台控制类型
 *
 * @return Enum_Gimbal_Control_Type 获取云台控制类型
 */
Enum_Gimbal_Control_Type Class_Gimbal::Get_Gimbal_Control_Type()
{
    return (Gimbal_Control_Type);
}

/**
 * @brief 设定云台状态
 *
 * @param __Gimbal_Control_Type 云台状态
 */
void Class_Gimbal::Set_Gimbal_Control_Type(Enum_Gimbal_Control_Type __Gimbal_Control_Type)
{
    Gimbal_Control_Type = __Gimbal_Control_Type;
}
/**
 * @brief 设定yaw轴角度
 *
 */
void Class_Gimbal::Set_Target_Yaw_Angle(float __Target_Yaw_Angle)
{
    Target_Yaw_Angle = __Target_Yaw_Angle;
}
/**
 * @brief 设定pitch轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_Angle(float __Target_Pitch_Angle)
{
    Target_Pitch_Angle = __Target_Pitch_Angle;
}
/**
 * @brief 设定pitch_2轴角度
 *
 */
void Class_Gimbal::Set_Target_Pitch_2_Angle(float __Target_Pitch_2_Angle)
{
    Target_Pitch_2_Angle = __Target_Pitch_2_Angle;
}
/**
 * @brief 设定roll轴角度
 *
 */
void Class_Gimbal::Set_Target_Roll_Angle(float __Target_Roll_Angle)
{
    Target_Roll_Angle = roll_cali_offset - __Target_Roll_Angle;
}

bool Class_FSM_Calibration::Get_roll_cali_status()
{
    return roll_cali_status;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
