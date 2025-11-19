/**
 * @file alg_power_limit.h
 * @author qyx
 * @brief 自适应功率限制算法
 * @version 1.2
 * @date
 *
 * @copyright ZLLC 2025
 *
 */

#include "alg_power_limit.h"
#include "math.h"

/* Private macros ------------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static inline bool floatEqual(float a, float b) { return fabs(a - b) < 1e-5f; }
static inline float rpm2av(float rpm) { return rpm * (float)PI / 30.0f; }
static inline float av2rpm(float av) { return av * 30.0f / (float)PI; }
static inline float my_fmax(float a, float b) { return (a > b) ? a : b; }

void Class_Power_Limit::Init()
{

    float initParams[2] = {k1, k2};
    rls.setParamVector(Matrixf<2, 1>(initParams));
}

/**
 * @brief 返回单个电机的计算功率
 *
 * @param omega 转子转速，单位为rpm
 * @param torque 转子扭矩大小，单位为nm
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机，舵轮需要传此参数
 * @return float 理论功率值
 */
//西交功率模型
float Class_Power_Limit::Calculate_Theoretical_Power(float omega, float torque, uint8_t motor_index)
{

    float cmdPower = rpm2av(omega) * torque +
                     fabs(rpm2av(omega)) * k1 +
                     torque * torque * k2 +
                     k3;

    return cmdPower;
}
//广工功率模型
float Class_Power_Limit::Calculate_Theoretical_Power(float omega[], float torque[], int number)
{
    float tmp_predict = 0;
    for (int i = 0; i < number; i++)
    {
				tmp_predict += fabs(rpm2av(omega[i]) * torque[i]) +
						k1 * torque[i] * torque[i] +
						k2 * rpm2av(omega[i]) * rpm2av(omega[i]) +
						k3;
    }
    return tmp_predict;
}
/**
 * @brief 计算功率系数
 *
 * @param actual_power 实际功率
 * @param motor_data 电机数据数组
 */
void Class_Power_Limit::Calculate_Power_Coefficient(float actual_power, float omega[], float torque[])
{

    static Matrixf<2, 1> samples;
    static Matrixf<2, 1> params;
    float effectivePower = 0;

    samples[0][0] = samples[1][0] = 0;

    if (actual_power > 5)
    {
        for (int i = 0; i < 4; i++)
        {
            effectivePower += torque[i] * rpm2av(omega[i]);

            samples[0][0] += rpm2av(omega[i]) * rpm2av(omega[i]);
            samples[1][0] += torque[i] * torque[i];
        }

        params = rls.update(samples, actual_power - effectivePower - 4 * k3);
        k1 = my_fmax(params[0][0], 1e-7f);
        k2 = my_fmax(params[1][0], 1e-7f);
    }
}

float Class_Power_Limit::Calculate_Limit_K(float omega[], float torque[], float power_limit, uint8_t motor_nums)
{
    float limit_k = 1.; // 输出伸缩因子k

    float tmp_predict = 0.0f;

    float torque_square_sum = 0.0f;
    float omega_square_sum = 0.0f;
    float torque_multi_omega_sum = 0.0f; 

    float func_a, func_b, func_c;
    float delta;

    for (int i = 0; i < motor_nums; i++)
    {
        torque_square_sum += torque[i] * torque[i];
        omega_square_sum += rpm2av(omega[i]) * rpm2av(omega[i]);
        torque_multi_omega_sum += fabs(rpm2av(omega[i]) * torque[i]);

        tmp_predict += fabs(rpm2av(omega[i]) * torque[i]) +
                       k1 * torque[i] * torque[i] +
                       k2 * rpm2av(omega[i]) * rpm2av(omega[i]) +
                       k3;
    }
    // power_predict = tmp_predict;

    if (tmp_predict > power_limit)
    {
        func_a = k1 * torque_square_sum;
        func_b = torque_multi_omega_sum;
        func_c = k2 * omega_square_sum - power_limit + k3 * motor_nums;

        delta = func_b * func_b - 4 * func_a * func_c; // b*b-4*a*c

        if (delta >= 0)
        {
            limit_k = (-func_b + sqrtf(delta)) / (2 * func_a); // 求根公式
        }
        else
        {
            limit_k = 0;
        }
    }

    return (limit_k > 1) ? 1 : limit_k;
}

/**
 * @brief 计算限制后的扭矩
 *
 * @param omega 转子转速，单位为rpm
 * @param power 限制功率值
 * @param torque 原始扭矩值
 * @param motor_index 电机索引，偶数为转向电机，奇数为动力电机
 * @return float 限制后的扭矩值
 */

uint8_t test_flag=0;
float solution1;
float solution2;
float test_cal;
float Class_Power_Limit::Calculate_Toque(float omega, float power, float torque, uint8_t motor_index)
{

    omega = rpm2av(omega);
    float newTorqueCurrent = 0.0f;

    float delta = omega * omega - 4 * (k1 * fabs(omega) + k3 - power) * k2;

    // if (torque * omega <= 0 || floatEqual(power, 0.0f))             //电机减速反向电动势是发出功率，不消耗功率
    // {
    //     newTorqueCurrent = torque;
    //     test_flag=0;
    // }
    // else
    // {
        if (floatEqual(delta, 0.0f))
        {
            newTorqueCurrent = -omega / (2.0f * k2);
            test_flag=1;
        }
        else if (delta > 0.0f)
        {
            float solution1 = (-omega + sqrtf(delta)) / (2.0f * k2);
            float solution2 = (-omega - sqrtf(delta)) / (2.0f * k2);

            newTorqueCurrent = (torque > 0) ? solution1 : solution2;

            test_flag=2;

            test_cal=(omega) * torque +
                     fabs((omega)) * k1 +
                     torque * torque * k2 +
                     k3;
        }
        else            //delta < 0
        {
            newTorqueCurrent = -omega / (2.0f * k2);
            test_flag=3;
        }
    // }
    return newTorqueCurrent;
}
/**
 * @brief 大P计算，计算每个电机分配的功率
 * @param Motor_Data 电机结构体
 * @param __Total_error 目标值与实际值绝对值误差加和
 * @param Max_Power 限制的最大功率
 * @param __Scale_Conffient 功率收缩因子
 */
void Class_Power_Limit::Calulate_Power_Allocate(Struct_Power_Motor_Data &Motor_Data, float __Total_error, float Max_Power, float __Scale_Conffient)
{
    Motor_Data.scaled_power = Motor_Data.theoretical_power *
                                    __Scale_Conffient;
    //按照误差分配
    // if(__Total_error <= ErrorLow ){
    //     Motor_Data.scaled_power = Motor_Data.theoretical_power *
    //                                 __Scale_Conffient;
    // }
    // else if(__Total_error >= ErrorUp){
    //     Motor_Data.scaled_power = Max_Power * 
    //                                 (Motor_Data.Target_error/__Total_error);
    // }
    // else{           //处于中间线性变化分配策略
    //     float Kp = 0.0f;
    //     Kp = (__Total_error-ErrorLow) / (ErrorUp - ErrorLow);

    //     Motor_Data.scaled_power =
    //         (1-Kp) * Motor_Data.theoretical_power *
    //         __Scale_Conffient   +    Kp * Max_Power * (Motor_Data.Target_error/__Total_error);
    // }
}
/**
 * @brief 功率限制主任务
 *
 * @param power_management 功率管理结构体
 */
float dirmotor_predic_power = 0.f,motmotor_predic_power = 0.f;
float dir_needScaled_power = 0.f,mot_needScaled_power = 0.f;
float dir_max_power = 0.0f,mot_max_power = 0.0f;
float power_pid_out = 0.0f,pre_pid_out = 0.0f;
void Class_Power_Limit::Power_Task(Struct_Power_Management &power_management)
{
    float tmp_dirmotor_predic_power = 0.0f,tmp_motmotor_predic_power = 0.0f;
    float tmp_dir_needScaled_power = 0.f,tmp_mot_needScaled_power = 0.f;

    for (int i = 0; i < 8; i++) 
    {
        if(i % 2 == 0)//转向电机
        {
            power_management.Motor_Data[i].theoretical_power = Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega, power_management.Motor_Data[i].torque, i);
            tmp_dirmotor_predic_power += power_management.Motor_Data[i].theoretical_power;
        }
        else//动力电机
        {
            power_management.Motor_Data[i].theoretical_power = Calculate_Theoretical_Power(power_management.Motor_Data[i].feedback_omega, power_management.Motor_Data[i].torque, i);
            tmp_motmotor_predic_power += power_management.Motor_Data[i].theoretical_power;
        }
        
        if(i % 2 == 0)
        {
            if(power_management.Motor_Data[i].theoretical_power > 0.0f)
            {
                tmp_dir_needScaled_power += power_management.Motor_Data[i].theoretical_power;
            }
            else
            {
                tmp_dir_needScaled_power += 0.0f;
            }
        }
        else
        {
             if(power_management.Motor_Data[i].theoretical_power > 0.0f)
            {
                tmp_mot_needScaled_power += power_management.Motor_Data[i].theoretical_power;
            }
            else
            {
                tmp_mot_needScaled_power += 0.0f;
            }
        }
    }
    if (Control_Status == 1)
    {
        power_pid_out = (power_management.Actual_Power - power_management.Max_Power + 1.f) * 5.0f;
        power_pid_out = (pre_pid_out + power_pid_out) / 2.0f;
        if (power_pid_out < 0.0f)
            power_pid_out = 0.0f;
        if (power_pid_out > 60.0f)
            power_pid_out = 60.0f;
        pre_pid_out = power_pid_out;
    }
    else
    {
        power_pid_out = 0;
    }
    //设置pid反馈值为0
    //power_pid_out = 0;
    dirmotor_predic_power = tmp_dirmotor_predic_power + power_pid_out;
    motmotor_predic_power = tmp_motmotor_predic_power + power_pid_out;
    dir_needScaled_power = tmp_dir_needScaled_power + power_pid_out;
    mot_needScaled_power = tmp_mot_needScaled_power + power_pid_out;
    //计算理论总功率
    power_management.Theoretical_Total_Power = dirmotor_predic_power + motmotor_predic_power;
    //计算需要收缩的理论总功率
    power_management.Needed_Scaled_Theoretical_Total_Power = dir_needScaled_power + mot_needScaled_power;

    //上层功率分配
    Power_Allocate(power_management.Max_Power,0.6f,dirmotor_predic_power,motmotor_predic_power,&dir_max_power,&mot_max_power);
    //下层功率限制
    if (dir_max_power >= dirmotor_predic_power) // 计算转向收缩系数
    {
        for (int j = 0; j < 8; j++)
        {
            if (j % 2 == 0)
                power_management.Motor_Data[j].output = power_management.Motor_Data[j].pid_output;
        }
    }
    else
    {
        power_management.Scale_Conffient[0] = dir_max_power / dir_needScaled_power;
        for (int i = 0; i < 8; i++)
        {
            if (i % 2 == 0)
            {
                if (power_management.Motor_Data[i].theoretical_power < 0.0f)
                {
                    power_management.Motor_Data[i].output = power_management.Motor_Data[i].pid_output;
                    continue;
                }

                Calulate_Power_Allocate(power_management.Motor_Data[i], 0,
                                        power_management.Max_Power, power_management.Scale_Conffient[0]);

                power_management.Motor_Data[i].output =
                    Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                                    power_management.Motor_Data[i].scaled_power,
                                    power_management.Motor_Data[i].torque,
                                    i) *
                    DIR_TORQUE_TO_CMD_CURRENT;
            }
        }
    }

    if (mot_max_power >= motmotor_predic_power) // 计算行进收缩系数
    {
        for (int j = 0; j < 8; j++)
        {
            if (j % 2 != 0)
                power_management.Motor_Data[j].output = power_management.Motor_Data[j].pid_output;
        }
    }
    else
    {
        power_management.Scale_Conffient[1] = mot_max_power / mot_needScaled_power;
        for (int i = 0; i < 8; i++)
        {
            if (i % 2 != 0)
            {
                if (power_management.Motor_Data[i].theoretical_power < 0.0f)
                {
                    power_management.Motor_Data[i].output = power_management.Motor_Data[i].pid_output;
                    continue;
                }

                Calulate_Power_Allocate(power_management.Motor_Data[i], 0,
                                        power_management.Max_Power, power_management.Scale_Conffient[1]);

                power_management.Motor_Data[i].output =
                    Calculate_Toque(power_management.Motor_Data[i].feedback_omega,
                                    power_management.Motor_Data[i].scaled_power,
                                    power_management.Motor_Data[i].torque,
                                    i) *
                    MOT_TORQUE_TO_CMD_CURRENT;
            }
        }
    }

    for (int i = 0; i < 8; i++)
    {
        if ((power_management.Motor_Data[i].output) >= 16384)
        {
            power_management.Motor_Data[i].output = 16384;
        }

		if ((power_management.Motor_Data[i].output) <= -16384)
        {
            power_management.Motor_Data[i].output = -16384;
        }
    }
}

void Class_Power_Limit::Power_Allocate(float Power_Limit, float rate,float dir_predict_power,float mot_predict_power,float *dir_power_allocate,float *mot_power_allocate)
{
    // 新的功率分配逻辑
    float dir_power_limit = Power_Limit * rate; // 转向电机功率上限
    float mot_power_limit = Power_Limit * (1.f-rate);      // 动力电机功率上限，动态计算

    //实际分配到的功率
    float dir_allocate,mot_allocate;

    // 首先分配转向电机功率
    if (dir_predict_power > dir_power_limit)
    {
        // 转向功率需求超过限制，按限制分配
        dir_allocate = dir_power_limit; 
    }
    else
    {
        // 转向功率需求未超限制，全部分配
        dir_allocate = dir_predict_power;
    }
    // 优先分配转向舵
    mot_power_limit = Power_Limit - dir_allocate;
    // 然后分配动力电机功率
    if (mot_predict_power > mot_power_limit)
    {
        mot_allocate = mot_power_limit;
    }
    else
    {
        mot_allocate = mot_predict_power;
    }
    //赋值给成员变量
    *dir_power_allocate = dir_allocate;
    *mot_power_allocate = mot_allocate;
}


void Class_Power_Limit::Power_Limit_Task()
{
    
}