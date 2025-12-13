

#ifndef _FLYING_SLOPE_H
#define _FLYING_SLOPE_H

#include "arm_math.h"
#include "drv_math.h"
#include "dvc_imu.h"
#include "dvc_djimotor.h"

//依据车体状态更改
#define Gravity        210                                         //整车重量*G == 重力 N
#define CHASSIS_L      0.18466f*2.0f                                         //底盘长  m
#define CHASSIS_W      0.18466f*2.0f                                         //宽      m
#define CHASSIS_R      0.154f/2.0f                                         //车轮半径 m

#define _M3508_REDUATION (3591.f / 187.f)                                                      // 3508标准减速比
#define _M3508_TORQUE_CONSTANT 0.3f                                                             // 3508带标准减速箱的转矩常数
#define _M3508_CMD_CURRENT_TO_TORQUE_CURRENT (20.f / 16384.f)                                  // Icmd映射到Itorque
#define _M3508_CMD_CURRENT_TO_TORQUE (_M3508_CMD_CURRENT_TO_TORQUE_CURRENT * _M3508_TORQUE_CONSTANT)          // 发送的电流控制值（16384）映射到输出轴扭矩

////对上坡，下坡进行的补偿（上坡加速，下坡减速）
class Class_Flying_Slope{
  public:
    Class_IMU *IMU;
    Class_DJI_Motor_C620 *Motor_Wheel[4];
    void Init(Class_DJI_Motor_C620 *__Motor_Wheel0, Class_DJI_Motor_C620 *__Motor_Wheel1,
      Class_DJI_Motor_C620 *__Motor_Wheel2, Class_DJI_Motor_C620 *__Motor_Wheel3);
    void Transform_Angle();
    void TIM_Calcualte_Feekback();
    void Output();

  protected:
    float Feekback_Value[4];    //0-3顺时针对应四个轮子
    float Slope_Angle = 0.0f;
    float Slope_Radian = 0.0f;
    float Roll_Angle = 0.0f;      //车体侧身角度

};


#endif
