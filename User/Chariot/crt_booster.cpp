/**
 * @file crt_booster.cpp
 * @author lez by wanghongxi
 * @brief 发射机构
 * @version 0.1
 * @date 2024-07-1 0.1 24赛季定稿
 *
 * @copyright ZLLC 2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 定时器处理函数
 * 这是一个模板, 使用时请根据不同处理情况在不同文件内重新定义
 *
 */
uint8_t Swtich_To_Angle_Control_Flag = 0;

#include <cmath>

/**
 * @brief 将角度归一化到 [0, 2π) 区间
 * @param angle 输入角度（弧度制）
 * @return 归一化后的角度，范围 [0, 2π)
 */
float convertToRange(float angle)
{
    constexpr float TWO_PI = 2.0f * PI;      // 2π 常量
    float result = std::fmod(angle, TWO_PI); // 取模运算
    if (result < 0.0f)
    {
        result += TWO_PI; // 负值调整到正区间
    }
    // 处理因浮点误差可能略大于等于 2π 的情况
    if (result >= TWO_PI)
    {
        result = 0.0f;
    }
    return result;
}

/**
 * @brief 将角度表示为 k*target + a，返回 a ∈ [0, target)
 * @param angle 输入角度（弧度制）
 * @param target 目标周期（弧度制，必须为正数）
 * @return 归一化后的偏差 a，范围 [0, target)
 */
float angleToTargetForm(float angle, float target)
{
    // 处理 target 为零的非法情况（可根据需求修改）
    if (target == 0.0f)
    {
        return angle; // 或返回 0.0f，或抛出异常
    }

    // 计算最大整数 k
    float k = std::floor(angle / target);
    // 计算余数 a
    float a = angle - k * target;

    // 浮点误差修正：确保 a 严格在 [0, target) 内
    if (a >= target)
    {
        a = 0.0f;
    }
    if (a < 0.0f)
    {
        a += target;
    }

    return a;
}

void Class_FSM_Heat_Detect::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态

        if (fabs(Booster->Motor_Friction_Right.Get_Now_Torque()) >= Booster->Friction_Torque_Threshold)
        {
            // 大扭矩->检测状态
            Set_Status(1);
        }
        else if (Booster->Booster_Control_Type == Booster_Control_Type_DISABLE)
        {
            // 停机->停机状态
            Set_Status(3);
        }
    }
    break;
    case (1):
    {
        // 发射嫌疑状态

        if (Status[Now_Status_Serial].Time >= 15)
        {
            // 长时间大扭矩->确认是发射了
            Set_Status(2);
        }
    }
    break;
    case (2):
    {
        // 发射完成状态->加上热量进入下一轮检测
        Booster->FiredCounter++;
        Heat += 10.0f;
        Set_Status(0);
    }
    break;
    case (3):
    {
        // 停机状态

        if (fabs(Booster->Motor_Friction_Right.Get_Now_Omega_Radian()) >= Booster->Friction_Omega_Threshold)
        {
            // 开机了->正常状态
            Set_Status(0);
        }
    }
    break;
    }

    // 热量冷却到0
    if (Heat > 0)
    {
        // Heat -= Booster->Referee->Get_Booster_17mm_1_Heat_CD() / 1000.0f;
    }
    else
    {
        Heat = 0;
    }
}

/**
 * @brief 卡弹策略有限自动机
 *
 */
void Class_FSM_Antijamming::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0):
    {
        // 正常状态
        Booster->Output();

        if (fabs(Booster->Motor_Driver.Get_Now_Torque()) >= Booster->Driver_Torque_Threshold)
        {
            // 大扭矩->卡弹嫌疑状态
            Set_Status(1);
        }
    }
    break;
    case (1):
    {
        // 卡弹嫌疑状态
        Booster->Output();

        if (Status[Now_Status_Serial].Time >= 100)
        {
            // 长时间大扭矩->卡弹反应状态
            Set_Status(2);
        }
        else if (fabs(Booster->Motor_Driver.Get_Now_Torque()) < Booster->Driver_Torque_Threshold)
        {
            // 短时间大扭矩->正常状态
            Set_Status(0);
        }
    }
    break;
    case (2):
    {
        // 卡弹反应状态->准备卡弹处理
        Booster->JammedCounter++;
        Booster->Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Drvier_Angle = Booster->Motor_Driver.Get_Now_Radian() + PI / 12.0f;
        Booster->Motor_Driver.Set_Target_Radian(Booster->Drvier_Angle);
        Set_Status(3);
    }
    break;
    case (3):
    {
        // 卡弹处理状态

        if (Status[Now_Status_Serial].Time >= 300)
        {
            // 长时间回拨->正常状态
            Set_Status(0);
        }
    }
    break;
    }
}
void Class_Booster_Driver::TIM_PID_PeriodElapsedCallback()
{
    switch (DJI_Motor_Control_Method)
    {
    case (DJI_Motor_Control_Method_OPENLOOP):
    {
        // 默认开环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_TORQUE):
    {
        // 默认闭环扭矩控制
        Out = Target_Torque / Torque_Max * Output_Max;
    }
    break;
    case (DJI_Motor_Control_Method_OMEGA):
    {
        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    case (DJI_Motor_Control_Method_ANGLE):
    {
        PID_Angle.Set_Target(Target_Radian);
        PID_Angle.Set_Now(Data.Now_Radian);
        PID_Angle.TIM_Adjust_PeriodElapsedCallback();

        Target_Omega_Radian = PID_Angle.Get_Out();

        PID_Omega.Set_Target(Target_Omega_Radian);
        PID_Omega.Set_Now(Data.Now_Omega_Radian);
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        Out = PID_Omega.Get_Out();
    }
    break;
    default:
    {
        Out = 0.0f;
    }
    break;
    }
    Output();
}

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    FiredCounter = 0;
    JammedCounter = 0;
    // 正常状态, 发射嫌疑状态, 发射完成状态, 停机状态
    FSM_Heat_Detect.Booster = this;
    FSM_Heat_Detect.Init(3, 3);

    // 正常状态, 卡弹嫌疑状态, 卡弹反应状态, 卡弹处理状态
    FSM_Antijamming.Booster = this;
    FSM_Antijamming.Init(4, 0);

    // 拨弹盘电机
    Motor_Driver.PID_Angle.Init(20.0f, 0.0f, 0.2f, 0.0f, 5.0f * PI, 5.0f * PI);
    Motor_Driver.PID_Angle.Set_I_Separate_Threshold(10.0f);
    Motor_Driver.PID_Omega.Init(1500.0f, 0.0f, 0.0f, 0.0f, Motor_Driver.Get_Output_Max(), Motor_Driver.Get_Output_Max());
    Motor_Driver.Init(&BOOSTER_CAN, DJI_Motor_ID_0x204, DJI_Motor_Control_Method_ANGLE, Fretboard_GEARBOX_RATIO);

    // 摩擦轮电机左
    Motor_Friction_Left.PID_Omega.Init(500.0f, 0.0f, 0.1f, 0.0f, 7000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Left.PID_Omega.Set_I_Separate_Threshold(30.0f);
    Motor_Friction_Left.Init(&BOOSTER_CAN, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机右
    Motor_Friction_Right.PID_Omega.Init(500.0f, 0.0f, 0.1f, 0.0f, 7000.0f, Motor_Friction_Right.Get_Output_Max());
    Motor_Friction_Right.PID_Omega.Set_I_Separate_Threshold(30.0f);
    Motor_Friction_Right.Init(&BOOSTER_CAN, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 摩擦轮电机下
    Motor_Friction_Down.PID_Omega.Init(500.0f, 0.0f, 0.1f, 0.0f, 7000.0f, Motor_Friction_Left.Get_Output_Max());
    Motor_Friction_Down.PID_Omega.Set_I_Separate_Threshold(30.0f);
    Motor_Friction_Down.Init(&BOOSTER_CAN, DJI_Motor_ID_0x206, DJI_Motor_Control_Method_OMEGA, 1.0f);
}
float Adjust_Radian = 0.0f;
bool Swtich_To_Chasefire_Control_Flag = 0; //=0为没有从连发状态切换为停火，=1为从连发状态切换为停火
/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
    // 控制拨弹轮
    switch (Booster_Control_Type)
    {
    case (Booster_Control_Type_DISABLE):
    {
        // 发射机构失能
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        // 关闭摩擦轮
        Set_Friction_Control_Type(Friction_Control_Type_DISABLE);

        Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
        Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Left.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Right.PID_Omega.Set_Integral_Error(0.0f);
        Motor_Friction_Down.PID_Omega.Set_Integral_Error(0.0f);

        Motor_Driver.Set_Out(0.0f);
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Down.Set_Target_Omega_Radian(0.0f);

        Motor_Driver.Set_Target_Radian(Motor_Driver.Get_Now_Radian());
    }
    break;
    case (Booster_Control_Type_CEASEFIRE):
    {
        // 停火
        if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_ANGLE)
        {
            // Motor_Driver.Set_Target_Angle(Motor_Driver.Get_Now_Angle());
        }
        else if (Motor_Driver.Get_Control_Method() == DJI_Motor_Control_Method_OMEGA)
        {

            Motor_Driver.Set_Target_Omega_Radian(0.0f);
            Drvier_Angle = Motor_Driver.Get_Now_Angle();
            Motor_Driver.PID_Angle.Set_Now(Motor_Driver.Get_Now_Angle());
            Motor_Driver.PID_Angle.Set_Integral_Error(0.0f);
            Motor_Driver.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Friction_Left.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Friction_Right.PID_Omega.Set_Integral_Error(0.0f);
            Motor_Friction_Down.PID_Omega.Set_Integral_Error(0.0f);
            float Tmp_Driver_Angle = Motor_Driver.Get_Now_Angle();
            Drvier_Angle = Motor_Driver.Get_Now_Radian();
            if (Swtich_To_Chasefire_Control_Flag)
            {
                Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
                
                //float Tmp_Drvier_Angle = convertToRange(Drvier_Angle);
                Adjust_Radian = angleToTargetForm(Drvier_Angle, 2.0f * PI / 9.0f);
                if (Adjust_Radian >= 0.03)
                    Adjust_Radian = (2.0f * PI / 9.0f) - angleToTargetForm(Drvier_Angle, 2.0f * PI / 9.0f);
                else
                    Adjust_Radian = 0.0f;
                Motor_Driver.Set_Target_Radian(Drvier_Angle + Adjust_Radian + (2.0f * PI / 9.0f));
                Swtich_To_Chasefire_Control_Flag = 0;
                
            }

        }
    }
    break;
    case (Booster_Control_Type_SINGLE):
    {
        // 单发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        if (Swtich_To_Angle_Control_Flag == 1)
        {
            Drvier_Angle = Motor_Driver.Get_Now_Radian();

            Swtich_To_Angle_Control_Flag = 0;
        }
        Drvier_Angle = Motor_Driver.Get_Now_Radian();
        Drvier_Angle += 2.0f * PI / 9.0f;
        Adjust_Radian = angleToTargetForm(Drvier_Angle, 2.0f * PI / 9.0f);
        if (Adjust_Radian >= 0.03)
            Adjust_Radian = (2.0f * PI / 9.0f) - angleToTargetForm(Drvier_Angle, 2.0f * PI / 9.0f);
        else
            Adjust_Radian = 0.0f;

        Motor_Driver.Set_Target_Radian(Drvier_Angle + Adjust_Radian);

        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;

        // #ifdef Heat_Detect_ENABLE
        //         if (FSM_Heat_Detect.Heat + 20 < Referee->Get_Booster_17mm_1_Heat_Max())
        //         {

        //             Drvier_Angle += 2.0f * PI / 9.0f;
        //             Motor_Driver.Set_Target_Radian(Drvier_Angle);
        //         }
        // #endif

        // #ifdef Heat_Detect_DISABLE
        //         Drvier_Angle += 2.0f * PI / 9.0f;
        //         Motor_Driver.Set_Target_Radian(Drvier_Angle);
        // #endif
        //         // 点一发立刻停火
        //         Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case (Booster_Control_Type_MULTI):
    {
        // 连发模式
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        // Drvier_Angle = Motor_Driver.Get_Now_Angle();
        Drvier_Angle += 2.0f * PI / 9.0f * 5.0f; // 五连发
        Motor_Driver.Set_Target_Radian(Drvier_Angle);

        // 点一发立刻停火
        Booster_Control_Type = Booster_Control_Type_CEASEFIRE;
    }
    break;
    case (Booster_Control_Type_REPEATED):
    {
        // 连发模式
        Motor_Driver.Set_Target_Omega_Radian(0.0f);
        Swtich_To_Angle_Control_Flag = 1;
        Swtich_To_Chasefire_Control_Flag = 1;
        Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Left.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Right.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Friction_Down.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Motor_Driver.Set_Target_Omega_Radian(-Driver_Omega);
        //Drvier_Angle = Motor_Driver.Get_Now_Radian();
    }
    break;
    }

    // 控制摩擦轮
    if (Friction_Control_Type != Friction_Control_Type_DISABLE)
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Right.Set_Target_Omega_Radian(Friction_Omega);
        Motor_Friction_Down.Set_Target_Omega_Radian(Friction_Omega);
    }
    else
    {
        Motor_Friction_Left.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Right.Set_Target_Omega_Radian(0.0f);
        Motor_Friction_Down.Set_Target_Omega_Radian(0.0f);
    }
}

/**
 * @brief 定时器计算函数
 *
 */
float speed = 0;
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{

    // 无需裁判系统的热量控制计算
    FSM_Heat_Detect.Reload_TIM_Status_PeriodElapsedCallback();
    // 卡弹处理
    FSM_Antijamming.Reload_TIM_Status_PeriodElapsedCallback();

    // Motor_Driver.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Driver.Set_Target_Omega_Radian(15);
    Motor_Driver.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Left.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Right.TIM_PID_PeriodElapsedCallback();
    Motor_Friction_Down.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
