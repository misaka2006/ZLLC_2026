/**
 * @file crt_chassis.cpp
 * @author cjw
 * @brief 底盘
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 轮组编号
 * 3 2
 *  1
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_chassis.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 底盘初始化
 *
 * @param __Speed 底盘速度限制最大值
 */
#ifdef TRACK_LEG
/**
 * @brief 底盘初始化
 *
 * @param __Speed 底盘速度限制最大值
 */
void Class_HybridTrackLeg_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    //Power_Limit.Init(400,3500);
    Supercap.Init(&hfdcan2,45.f);

    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.005f,0.01f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.005f,0.01f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f, 0.05f);

    //imu初始化
    BoardDM_BMI.Init();

    //轮向电机PID初始化
    Motor_Wheel[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[0].Get_Output_Max(), Motor_Wheel[0].Get_Output_Max());
    Motor_Wheel[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[1].Get_Output_Max(), Motor_Wheel[1].Get_Output_Max());
    Motor_Wheel[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[2].Get_Output_Max(), Motor_Wheel[2].Get_Output_Max());
    Motor_Wheel[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[3].Get_Output_Max(), Motor_Wheel[3].Get_Output_Max());
    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x204);

    //关节电机PID初始化
    //关节电机ID初始化
    Motor_Joint[0].Init(&hfdcan2, DM_Motor_ID_0xA1, DM_Motor_Control_Method_POSITION_OMEGA);
    Motor_Joint[1].Init(&hfdcan2, DM_Motor_ID_0xA2, DM_Motor_Control_Method_POSITION_OMEGA);

    //履带驱动电机PID初始化
    Motor_Track[0].PID_Omega.Init(3000.0f, 6.0f, 0.0f, 0.0f, Motor_Track[0].Get_Output_Max(), Motor_Track[0].Get_Output_Max());
    Motor_Track[1].PID_Omega.Init(10.0f, 0.0f, 0.0f, 0.0f, Motor_Track[1].Get_Output_Max(), Motor_Track[1].Get_Output_Max());//需调参
    //履带电机ID初始化
    Motor_Track[0].Init(&hfdcan2,DJI_Motor_ID_0x201);
    Motor_Track[1].Init(&hfdcan2,DJI_Motor_ID_0x202);

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
}
#endif
#ifdef AGV
/**
 * @brief 底盘初始化
 *
 * @param __Speed 底盘速度限制最大值
 */
void Class_Streeing_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max, float __Steer_Power_Ratio)
{
    //Power_Limit.Init(400,3500);
    Supercap.Init(&hfdcan2,100.f);
    
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;
    Steer_Power_Ratio = __Steer_Power_Ratio;

    //斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.01f,0.025f);
    //斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.01f,0.025f);
    //斜坡函数加减速角速度
    Slope_Omega.Init(0.05f,0.05f);

    #ifdef OLD
    //电机PID批量初始化
    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, Motor_Wheel[i].Get_Output_Max(), Motor_Wheel[i].Get_Output_Max());
    }
    #ifdef AGV
    //轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan2, DJI_Motor_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan2, DJI_Motor_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan2, DJI_Motor_ID_0x203);
    Motor_Wheel[3].Init(&hfdcan2, DJI_Motor_ID_0x204);
    
    //舵向电机PID初始化

    Motor_Steer[0].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[0].Get_Output_Max(), Motor_Steer[0].Get_Output_Max());
    Motor_Steer[0].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[0].Get_Output_Max());
    
    Motor_Steer[1].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[1].Get_Output_Max(), Motor_Steer[1].Get_Output_Max());
    Motor_Steer[1].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[1].Get_Output_Max());

    Motor_Steer[2].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[2].Get_Output_Max(), Motor_Steer[2].Get_Output_Max());
    Motor_Steer[2].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[2].Get_Output_Max());

    Motor_Steer[3].PID_Angle.Init(10.f, 0.0f, 0.0f, 0.0f, Motor_Steer[3].Get_Output_Max(), Motor_Steer[3].Get_Output_Max());
    Motor_Steer[3].PID_Omega.Init(1000.0f, 0.0f, 0.0f, 0.0f, 8000, Motor_Steer[3].Get_Output_Max());


    //舵向电机ID初始化
    Motor_Steer[0].Init(&hfdcan2, DJI_Motor_ID_0x205);
    Motor_Steer[1].Init(&hfdcan2, DJI_Motor_ID_0x206);
    Motor_Steer[2].Init(&hfdcan2, DJI_Motor_ID_0x207);
    Motor_Steer[3].Init(&hfdcan2, DJI_Motor_ID_0x208);
    //舵向电机零点位置初始化
    Motor_Steer[0].Set_Zero_Position(1.76999998);               //应该是轮子朝向的正方向
    Motor_Steer[1].Set_Zero_Position(2.38000011);
    Motor_Steer[2].Set_Zero_Position(2.16000009);
    Motor_Steer[3].Set_Zero_Position(2.6400001);
    #endif

    #ifdef OMNI_WHEEL
        Motor_Wheel[0].Init(&hfdcan1, DJI_Motor_ID_0x203);
        Motor_Wheel[1].Init(&hfdcan1, DJI_Motor_ID_0x201);
        Motor_Wheel[2].Init(&hfdcan1, DJI_Motor_ID_0x204);
        Motor_Wheel[3].Init(&hfdcan1, DJI_Motor_ID_0x202);
    #endif

    //底盘控制方式初始化
    Chassis_Control_Type = Chassis_Control_Type_DISABLE;
    #endif
}
#endif

/**
 * @brief 速度解算
 *
 */
float car_V,car_yaw;//车体总体朝向与速度
#ifdef TRACK_LEG
/**
 * @brief 速度解算
 *
 */
void Class_HybridTrackLeg_Chassis::Speed_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case(Chassis_Control_Type_DISABLE):
    {
        for(int i = 0; i < 4;i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OPENLOOP);
            Motor_Wheel[i].PID_Omega.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
        break;
    }
    case(Chassis_Control_Type_SPIN_Positive):
    case(Chassis_Control_Type_SPIN_Negative):
    case(Chassis_Control_Type_FLLOW):
    {
        //电机模式配置
        //轮向电机
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        }
        
        //底盘限速
        if (Velocity_X_Max != 0)
        {
            Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
        }
        if (Velocity_Y_Max != 0)
        {
            Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
        }
        if (Omega_Max != 0)
        {
            Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
        }
        #ifdef SPEED_SLOPE
        //速度换算，正运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out()*(HALF_WIDTH+HALF_LENGTH);
        #else
        //速度换算，正运动学分解
        float motor1_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        float motor2_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        float motor3_temp_linear_vel = Target_Velocity_Y + Target_Velocity_X + Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        float motor4_temp_linear_vel = Target_Velocity_Y - Target_Velocity_X - Target_Omega*(HALF_WIDTH+HALF_LENGTH);
        #endif            
        //线速度 cm/s  转角速度  RAD 
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;
        float motor4_temp_rad = motor4_temp_linear_vel * VEL2RAD;
        //角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega_Radian(  motor2_temp_rad);
        Motor_Wheel[1].Set_Target_Omega_Radian(- motor1_temp_rad);
        Motor_Wheel[2].Set_Target_Omega_Radian(- motor3_temp_rad);
        Motor_Wheel[3].Set_Target_Omega_Radian(  motor4_temp_rad);

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
        }
        break;
    }
    }
}
#endif

#ifdef AGV
/**
 * @brief 速度解算
 *
 */
void Class_Streeing_Chassis::Speed_Resolution(){ 
    #ifdef AGV 
   switch (Chassis_Control_Type)
    {
        case(Chassis_Control_Type_DISABLE):
        {
            // 底盘失能 四舵轮输出转矩为0
            break;
        }
        case (Chassis_Control_Type_SPIN_Positive):
        case (Chassis_Control_Type_SPIN_Negative):
        case(Chassis_Control_Type_FLLOW):
        {
            // 底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);
            }
            // 速度换算，正运动学分解
            // 右下位置的舵轮
            float Wheel_A_Vx = Target_Velocity_X - Target_Omega * R_A * sin(THETA);
            float Wheel_A_Vy = Target_Velocity_Y - Target_Omega * R_A * cos(THETA);
            // 左下位置的舵轮
            float Wheel_B_Vx = Target_Velocity_X - Target_Omega * R_B * sin(THETA);
            float Wheel_B_Vy = Target_Velocity_Y + Target_Omega * R_B * cos(THETA);
            // 左上位置的舵轮
            float Wheel_C_Vx = Target_Velocity_X + Target_Omega * R_C * sin(THETA);
            float Wheel_C_Vy = Target_Velocity_Y + Target_Omega * R_C * cos(THETA);
            // 右上位置的舵轮
            float Wheel_D_Vx = Target_Velocity_X + Target_Omega * R_D * sin(THETA);
            float Wheel_D_Vy = Target_Velocity_Y - Target_Omega * R_D * cos(THETA);
            #ifdef SPEED_SLOPE
            // 速度换算，正运动学分解
            // 右下位置的舵轮
            float Wheel_A_Vx = Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * R_A * sin(THETA);
            float Wheel_A_Vy = Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * R_A * cos(THETA);
            // 左下位置的舵轮
            float Wheel_B_Vx = Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * R_B * sin(THETA);
            float Wheel_B_Vy = Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * R_B * cos(THETA);
            // 左上位置的舵轮
            float Wheel_C_Vx = Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * R_C * sin(THETA);
            float Wheel_C_Vy = Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * R_C * cos(THETA);
            // 右上位置的舵轮
            float Wheel_D_Vx = Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * R_D * sin(THETA);
            float Wheel_D_Vy = Slope_Velocity_Y.Get_Out() - Slope_Omega.Get_Out() * R_D * cos(THETA);
            #endif

            wheel[0].ChassisCoordinate_Angle = My_atan(Wheel_A_Vy, Wheel_A_Vx) * RAD_TO_8191;
            wheel[1].ChassisCoordinate_Angle = My_atan(Wheel_B_Vy, Wheel_B_Vx) * RAD_TO_8191;
            wheel[2].ChassisCoordinate_Angle = My_atan(Wheel_C_Vy, Wheel_C_Vx) * RAD_TO_8191;
            wheel[3].ChassisCoordinate_Angle = My_atan(Wheel_D_Vy, Wheel_D_Vx) * RAD_TO_8191;

            wheel[0].streeing_wheel_speed = sqrt(Square(Wheel_A_Vx) + Square(Wheel_A_Vy));
            wheel[1].streeing_wheel_speed = sqrt(Square(Wheel_B_Vx) + Square(Wheel_B_Vy));
            wheel[2].streeing_wheel_speed = sqrt(Square(Wheel_C_Vx) + Square(Wheel_C_Vy));
            wheel[3].streeing_wheel_speed = sqrt(Square(Wheel_D_Vx) + Square(Wheel_D_Vy));

            for (auto i = 0; i < 4; i++)
            {
                wheel[i].streeing_wheel_omega = wheel[i].streeing_wheel_speed * VEL2RPM;
            }
        }
        #ifdef OLD
        case(Chassis_Control_Type_SPIN):
        {
            
            //轮组自锁，每个小轮坐标系都符合右手系
            static uint32_t Lock_Time = 0;
            static uint8_t  Lock_Flag = 0;
            float delta_Angle = 0.0f, Transform_Radian = 0.0f;                  //用于优化处理的变量
            if (fabs(Target_Velocity_X) < 0.01 && fabs(Target_Velocity_Y) < 0.01 && fabs(Target_Omega) < 0.01)
            {
                
                Lock_Time++;
                if(Lock_Time > 100)  Lock_Flag = 1;
                if (Lock_Flag)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                        Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE); // 舵轮控制模式

                        Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                    }

                    Motor_Steer[0].Set_Target_Radian(-PI / 4.0f);
                    Motor_Steer[1].Set_Target_Radian( PI / 4.0f);
                    Motor_Steer[2].Set_Target_Radian(-PI / 4.0f);
                    Motor_Steer[3].Set_Target_Radian( PI / 4.0f);
                    for (int i = 0; i < 4; i++)
                    {
                        Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();

                        //优劣弧处理
                        if((i % 2) == 0){
                            delta_Angle = - PI / 4.0f - Transform_Radian;
                        }
                        else{
                            delta_Angle = PI / 4.0f - Transform_Radian;
                        }
                        delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);

                        Motor_Steer[i].Set_Target_Radian(Transform_Radian + delta_Angle);
                        Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                        // Motor_Steer[i].Set_Out(0.0f);
                        // Motor_Wheel[i].Set_Out(0.0f);
                        Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
                        Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                    }
                    break;
                    
                }
            }

            if(Lock_Flag){
                Lock_Flag = 0;
                Lock_Time = 0;
            }

            //0 1 2 3 左前 右前 右后 左后 逆时针    前X左Y坐标系   基于编码器0度朝前，逆时针为正角度   确保轮子正转的是朝前的速度，不然得单独加负号
            float True_Vx[4], True_Vy[4], True_Target_Angle_Radian[4];
            
            //斜坡处理
            True_Vx[0] = True_Vx[3] = Slope_Velocity_X.Get_Out() - sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;
            True_Vx[1] = True_Vx[2] = Slope_Velocity_X.Get_Out() + sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;

            True_Vy[0] = True_Vy[1] = Slope_Velocity_Y.Get_Out() + sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;
            True_Vy[2] = True_Vy[3] = Slope_Velocity_Y.Get_Out() - sqrt(2) * Slope_Omega.Get_Out() *  CHASSIS_RADIUS/ 2;
            

            // True_Vx[0] = True_Vx[3] = Target_Velocity_X - sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;
            // True_Vx[1] = True_Vx[2] = Target_Velocity_X + sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;

            // True_Vy[0] = True_Vy[1] = Target_Velocity_Y + sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;
            // True_Vy[2] = True_Vy[3] = Target_Velocity_Y - sqrt(2) * Target_Omega *  CHASSIS_RADIUS/ 2;

            //舵轮转动角度的优化处理
            for(int i = 0;i<4;i++){
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Steer[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_AGV_MODE);         //舵轮控制模式

                //计算速度
                float temp_Target_Omega = 0.0f;
                arm_sqrt_f32(True_Vx[i] * True_Vx[i] + True_Vy[i] * True_Vy[i], &temp_Target_Omega);
                temp_Target_Omega = temp_Target_Omega / WHEEL_RADIUS;

                //计算目标角度
                if(fabs(temp_Target_Omega) < 0.0001){            //避免X =0 ；Y = 0的情况
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                }
                else{
                    True_Target_Angle_Radian[i] = atan2f(True_Vy[i], True_Vx[i]);           //-PI -- PI   会自动处理Vx = 0;
                }
                
                //角度优化处理
                delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian();     // -2PI -- 2PI  
                delta_Angle = Normalize_Angle_Radian_PI_to_PI(delta_Angle);                 // 处理重叠的角度（-20 = 340），归一化到 -PI --- PI
                if(delta_Angle > PI/2.0f){
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle - PI;
                    temp_Target_Omega *= -1.0f;
                }
                else if(delta_Angle < -PI/2.0f){
                    True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + delta_Angle + PI;
                    temp_Target_Omega *= -1.0f;
                }
                else{
                    //不需要处理角度
                }
                
                //处理-180 - 180的突变问题    同时还有优劣弧处理
                delta_Angle = True_Target_Angle_Radian[i] - Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                True_Target_Angle_Radian[i] = Motor_Steer[i].Get_Now_Zero_Offset_Radian() + Normalize_Angle_Radian_PI_to_PI(delta_Angle);
                
                Motor_Steer[i].Set_Target_Radian(True_Target_Angle_Radian[i]);
                Motor_Wheel[i].Set_Target_Omega_Radian(temp_Target_Omega);
            }

            for(int i=0;i<4;i++)
            {   
                Transform_Radian = Motor_Steer[i].Get_Now_Zero_Offset_Radian();
                Motor_Steer[i].Set_Transform_Radian(Transform_Radian);
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
                Motor_Steer[i].TIM_PID_PeriodElapsedCallback();
            }
            break;
            
        }
        #endif
    }
    #endif   
    #ifdef OMNI_WHEEL
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            //底盘失能 轮组无力
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
                Motor_Wheel[i].Set_Target_Omega_Radian(0.0f);
                Motor_Wheel[i].Set_Out(0.0f);
            }            
        }
        break;
        case (Chassis_Control_Type_FLLOW) :
        {
            // 底盘四电机模式配置
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }
            //底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Omega_Max, -Omega_Max, Omega_Max);
            }

            // 速度换算，正运动学分解
            float motor1_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
            float motor2_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
            float motor3_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
            float motor4_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);

            // 斜坡函数
            // float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
            // float motor2_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
            // float motor3_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Velocity_X.Get_Out() + Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);
            // float motor4_temp_linear_vel = Slope_Velocity_Y.Get_Out() - Slope_Velocity_X.Get_Out() - Slope_Omega.Get_Out() * (HALF_WIDTH + HALF_LENGTH);

            // 线速度 cm/s  转角速度  RAD
            float motor1_temp_rad = motor1_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor2_temp_rad = motor2_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor3_temp_rad = motor3_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor4_temp_rad = motor4_temp_linear_vel * VEL2RPM * RPM2RAD;
            // 角速度*减速比  设定目标
            Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad * M3508_REDUCTION_RATIO);

            for(int i=0;i<4;i++)
            {
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            }
        }
        break;
        case (Chassis_Control_Type_SPIN) :
        {
            // 底盘四电机模式配置
            for (int i = 0; i < 4; i++)
            {
                Motor_Wheel[i].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            }
            //底盘限速
            if (Velocity_X_Max != 0)
            {
                Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);
            }
            if (Velocity_Y_Max != 0)
            {
                Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);
            }
            if (Omega_Max != 0)
            {
                Math_Constrain(&Omega_Max, -Omega_Max, Omega_Max);
            }

            // 速度换算，正运动学分解
            float motor1_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
            float motor2_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
            float motor3_temp_linear_vel = Get_Target_Velocity_Y() + Get_Target_Velocity_X() + Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);
            float motor4_temp_linear_vel = Get_Target_Velocity_Y() - Get_Target_Velocity_X() - Get_Target_Omega() * (HALF_WIDTH + HALF_LENGTH);

            // 线速度 cm/s  转角速度  RAD
            float motor1_temp_rad = motor1_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor2_temp_rad = motor2_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor3_temp_rad = motor3_temp_linear_vel * VEL2RPM * RPM2RAD;
            float motor4_temp_rad = motor4_temp_linear_vel * VEL2RPM * RPM2RAD;
            // 角速度*减速比  设定目标
            Motor_Wheel[0].Set_Target_Omega_Radian(motor2_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[1].Set_Target_Omega_Radian(-motor1_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[2].Set_Target_Omega_Radian(-motor3_temp_rad * M3508_REDUCTION_RATIO);
            Motor_Wheel[3].Set_Target_Omega_Radian(motor4_temp_rad * M3508_REDUCTION_RATIO);

            for(int i=0;i<4;i++)
            {
                Motor_Wheel[i].TIM_PID_PeriodElapsedCallback();
            }
        }
        break;
    }
    #endif
}
#endif

#ifdef TRACK_LEG
/**
 * @brief 姿态切换函数
 * 
 */
void Class_HybridTrackLeg_Chassis::Switch_Pose()
{
    #ifdef LOCKED_SWITCH
    switch (Pose_Control_Type)
    {
    case(Pose_DISABLE)://失能
    {
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        break;
    }
    case(Pose_STANDBY)://待机
    {
        //设定关节角度
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[0].Set_Target_Angle(0.0f);
        Motor_Joint[0].Set_Target_Omega(1.0f);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_Target_Angle(0.0f);
        Motor_Joint[1].Set_Target_Omega(1.0f);
        break;
    }
    case(Pose_ENABLE)://使能
    {
        //设置关节角度
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[0].Set_Target_Angle(PI /2);
        Motor_Joint[0].Set_Target_Omega(1.0f);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_Target_Angle(PI /2);
        Motor_Joint[1].Set_Target_Omega(1.0f);
        break;
    }
    }
    if(Pose_Control_Type != Pose_DISABLE)
    {
        //开启履带驱动电机
        Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[0].Set_Target_Omega_Radian(5.0f);
        Motor_Track[1].Set_Target_Omega_Radian(5.0f);
    }
    else
    {
        //关闭履带驱动电机
        Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Motor_Track[0].Set_Target_Omega_Radian(0.0f);
        Motor_Track[1].Set_Target_Omega_Radian(0.0f);
    }
    #endif
    #ifdef AUTO_SWITCH //自动伸缩腿，已注释，不过目前是开环，后期可做成闭环
    static uint16_t mod2s = 0;// 2s重置计数器
    static uint8_t  pose_state = 1; // 位姿控制状态 0-Enable 1-Standby 
    Chassis_Pitch = BoardDM_BMI.Get_Angle_Pitch();
    Error_Pitch = Chassis_Pitch;
    // if (Error_Pitch > 10.0f)
    // {
    //     // 如果误差大于 10°，切换到 ENABLE
    //     if (pose_state != 0)
    //     {
    //         Set_Pose_Control_Type(Pose_ENABLE);
    //         pose_state = 0;
    //         mod2s = 0;  // 重置计时器
    //     }
    // }
    // else
    // {
    //     // 如果误差小于等于 10°，检查是否需要切换回 STANDBY
    //     if (pose_state == 0)
    //     {
    //         // 如果当前在 ENABLE 状态，计时器累加
    //         mod2s++;
    //         if (mod2s >= 2000)  // 2000ms = 2s
    //         {
    //             Set_Pose_Control_Type(Pose_STANDBY);
    //             pose_state = 1;
    //             mod2s = 0;  // 重置计时器
    //         }
    //     }
    // }
    switch (Pose_Control_Type)
    {
        case(Pose_DISABLE)://失能
    {
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_DISABLE);
        break;
    }
    case(Pose_STANDBY)://待机
    {
        //启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        //设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        //MIT模式
        // for(int i = 0; i <2; i++)
        // {
        //     Motor_Joint[i].Set_Target_Angle(0.0f);
        //     Motor_Joint[i].Set_Target_Omega(0.0f);
        //     // Motor_Joint[i].Set_Target_Torque(0.0f);//需要测试腿克服重力所需要的前馈力矩
        //     // Motor_Joint[i].Set_MIT_K_P(15.0f);//还需测试合适的数值
        //     // Motor_Joint[i].Set_MIT_K_D(1.0f);
        // }
        //位置速度模式
        Motor_Joint[0].Set_Target_Angle(0.0f);
        Motor_Joint[0].Set_Target_Omega(1.0f);
        Motor_Joint[1].Set_Target_Angle(0.0f);
        Motor_Joint[1].Set_Target_Omega(-1.0f);
        break;
    }
    case(Pose_ENABLE)://使能
    {
        //启动控制方式
        Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        //设定控制帧所需参数： 角度、角速度、t_ff、Kp、Kd
        //位置速度模式
        Motor_Joint[0].Set_Target_Angle(PI / 3);
        Motor_Joint[0].Set_Target_Omega(1.0f);
        Motor_Joint[1].Set_Target_Angle(-PI / 3);
        Motor_Joint[1].Set_Target_Omega(-1.0f);
        //MIT模式
        // Motor_Joint[0].Set_Target_Angle(PI / 3);
        // Motor_Joint[0].Set_Target_Omega(0.0f);
        // Motor_Joint[0].Set_Target_Torque(0.0f);//需要测试腿克服重力所需要的前馈力矩
        // Motor_Joint[0].Set_MIT_K_P(15.0f);//需测试
        // Motor_Joint[0].Set_MIT_K_D(1.0f);
        // Motor_Joint[1].Set_Target_Angle(-PI / 3);
        // Motor_Joint[1].Set_Target_Omega(0.0f);
        // Motor_Joint[1].Set_Target_Torque(0.0f);//需要测试腿克服重力所需要的前馈力矩
        // Motor_Joint[1].Set_MIT_K_P(15.0f);//需测试
        // Motor_Joint[1].Set_MIT_K_D(1.0f);
        break;
    }
    }
    #endif
    Motor_Joint[0].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    Motor_Joint[1].Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
    //计算回调函数
    for(int i = 0; i < 2; i++)
    {
        Motor_Joint[i].TIM_Process_PeriodElapsedCallback();
        Motor_Track[i].TIM_PID_PeriodElapsedCallback();
    }
}
#endif

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
#ifdef TRACK_LEG
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_HybridTrackLeg_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    //debug
    //Chassis_Control_Type = Enum_Chassis_Control_Type(Chassis_Flag);
    #ifdef SPEED_SLOPE
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();

    #endif
    //速度解算
    // Speed_Resolution();
    //位姿切换
    Switch_Pose();
    #if POWER_CONTROL == 1
    /*************************功率限制策略*******************************/
    //1.获取电机信息
    uint8_t chassis_status = (uint8_t)(Get_Chassis_Control_Type());
    // 清空不存在的转向电机
    for (uint8_t i = 0; i < 8; i += 2) 
    {
    Power_Management.Motor_Data[i].feedback_omega = 0;
    Power_Management.Motor_Data[i].feedback_torque = 0;
    Power_Management.Motor_Data[i].pid_output = 0;
    Power_Management.Motor_Data[i].torque = 0;
    }
    Power_Management.Motor_Data[1].feedback_omega = Motor_Wheel[0].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[3].feedback_omega = Motor_Wheel[1].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[5].feedback_omega = Motor_Wheel[2].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[7].feedback_omega = Motor_Wheel[3].Get_Now_Omega_Radian() * M3508_REDUATION * RAD_TO_RPM;
    Power_Management.Motor_Data[1].feedback_torque = Motor_Wheel[0].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[3].feedback_torque = Motor_Wheel[1].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[5].feedback_torque = Motor_Wheel[2].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[7].feedback_torque = Motor_Wheel[3].Get_Now_Torque() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[1].pid_output = Motor_Wheel[0].Get_Out();
    Power_Management.Motor_Data[3].pid_output = Motor_Wheel[1].Get_Out();
    Power_Management.Motor_Data[5].pid_output = Motor_Wheel[2].Get_Out();
    Power_Management.Motor_Data[7].pid_output = Motor_Wheel[3].Get_Out();
    Power_Management.Motor_Data[1].torque = Motor_Wheel[0].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[3].torque = Motor_Wheel[1].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[5].torque = Motor_Wheel[2].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    Power_Management.Motor_Data[7].torque = Motor_Wheel[3].Get_Out() * M3508_CMD_CURRENT_TO_TORQUE;
    //2.跑功率控制策略
    Power_Limit.Set_Control_Status(chassis_status);
    Power_Management.Max_Power = 100.0f;

    Power_Limit.Power_Task(Power_Management);
    //3.获取输出
    Motor_Wheel[0].Set_Out(Power_Management.Motor_Data[1].output);
    Motor_Wheel[1].Set_Out(Power_Management.Motor_Data[3].output);
    Motor_Wheel[2].Set_Out(Power_Management.Motor_Data[5].output);
    Motor_Wheel[3].Set_Out(Power_Management.Motor_Data[7].output);
    #endif
}
#endif

#ifdef AGV
void Class_Streeing_Chassis::AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate()
{
    for (auto i = 0; i < 4; i++)
    {
        wheel[i].streeing_wheel_angle = wheel[i].ChassisCoordinate_Angle;
    }
    if (break_mode)
    {
        wheel[0].streeing_wheel_angle = 8191 * (215) / 360;
        wheel[1].streeing_wheel_angle = 8191 * (135) / 360;
        wheel[2].streeing_wheel_angle = 8191 * (45) / 360;
        wheel[3].streeing_wheel_angle = 8191 * (315) / 360;
    }
    for (auto i = 0; i < 4; i++)
    {
        
        if (wheel[i].streeing_wheel_angle > 8191)
            wheel[i].streeing_wheel_angle -= 8191;
        else if (wheel[i].streeing_wheel_angle < 0)
            wheel[i].streeing_wheel_angle += 8191;
    }
}

void Class_Streeing_Chassis::Speed_Limitation()
{
    float temp = 0;
    temp = fabsf(wheel[0].streeing_wheel_omega);
    if (fabsf(wheel[1].streeing_wheel_omega) > temp)
        temp = fabsf(wheel[1].streeing_wheel_omega);
    if (fabsf(wheel[2].streeing_wheel_omega) > temp)
        temp = fabsf(wheel[2].streeing_wheel_omega);
    if (fabsf(wheel[3].streeing_wheel_omega) > temp)
        temp = fabsf(wheel[3].streeing_wheel_omega);
    if (temp > MAX_MOTOR_SPEED)
    {
        for (auto i = 0; i < 4; i++)
        {
            wheel[i].streeing_wheel_omega = (wheel[i].streeing_wheel_omega * MAX_MOTOR_SPEED * 1.0f / temp);
        }
    }
    else
    {
        for (auto i = 0; i < 4; i++)
        {
            wheel[i].streeing_wheel_omega = (wheel[i].streeing_wheel_omega);
        }
    }
}

void Class_Streeing_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    static uint16_t time_cnt = 0;

    // 1.底盘四舵轮驻车模式
    // 2.随动模式不进行移动状态下，为了正确设置四舵轮角度
    if ((Target_Velocity_X == 0) && (Target_Velocity_Y == 0) && (Target_Omega == 0))
    {
        break_mode = 1;
    }
    else
    {
        break_mode = 0;
    }
    
    #ifdef SPEED_SLOPE
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    #endif
    
    Speed_Resolution();
    AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate();
    Speed_Limitation();
}
#endif

#ifdef OLD
/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Tricycle_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    //斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    
    //速度解算
    Speed_Resolution();    
    /***************************超级电容*********************************/
    Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max());
    Supercap.Set_Supercap_Mode(Get_Supercap_Mode());
    Supercap.TIM_Supercap_PeriodElapsedCallback();

    #if POWER_CONTROL == 1
    /*************************功率限制策略*******************************/
    //Power_Limit_Update();
    Power_Limit.Set_Motor(Motor_Wheel);//添加四个电机的控制电流和当前转速
    Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max());
    if(Referee->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)
    {
        if(Supercap.Get_Consuming_Power() <= 200.f)
        {
            Power_Limit.Set_Power_Limit(Referee->Get_Chassis_Power_Max()/3.0f);
            Supercap.Set_Limit_Power(Referee->Get_Chassis_Power_Max()/3.0f);
        }
    }
    Power_Limit.Set_Chassis_Buffer(Referee->Get_Chassis_Energy_Buffer());
    Power_Limit.TIM_Adjust_PeriodElapsedCallback(Motor_Wheel);
    #endif
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
