/**
 * @file ita_chariot.cpp
 * @author cjw by yssickjgd
 * @brief 人机交互控制逻辑
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "ita_chariot.h"
#include "drv_math.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 控制交互端初始化
 *
 */
void Class_Chariot::Init(float __DR16_Dead_Zone)
{
    #ifdef CHASSIS
    
        //裁判系统
        Referee.Init(&huart10);
        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);
        //遥控器
        DR16.Init(&huart5,&huart1);
        DR16_Dead_Zone = __DR16_Dead_Zone;
        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();
        Force_Control_Chassis.Init();
        #ifdef AGV
        // 底盘随动PID环初始化
        PID_Chassis_Fllow.Init(6.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.0f, 0.001f); // Kp=3
        // yaw电机canid初始化  只获取其编码器值用于底盘随动，并不参与控制
        Motor_Yaw.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE, 2);
        #endif
        //超电
        Chassis.Supercap.Referee = &Referee;

    #elif defined(GIMBAL)
        
        Chassis.Set_Velocity_X_Max(4.0f);
        Chassis.Set_Velocity_Y_Max(4.0f);

        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);

        //遥控器
        #ifdef USE_DR16
        DR16.Init(&huart5,&huart1);
        DR16_Dead_Zone = __DR16_Dead_Zone;   
        #endif

        #ifdef USE_VT13
        FSM_Alive_Control_VT13.Chariot = this;
        FSM_Alive_Control_VT13.Init(5,0);
        #endif

        //云台
        Gimbal.Init();
        Gimbal.MiniPC = &MiniPC;

        //发射机构
        Booster.Init();
        Booster.MiniPC = &MiniPC;
        Booster.Referee = &Referee;

        //裁判系统
        Referee.Init(&huart10);
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object,&UART8_Manage_Object,&CAN3_Manage_Object);
        MiniPC.IMU = &Gimbal.Boardc_BMI;
        MiniPC.Referee = &Referee;

        //底盘随动环pid初始化(角度结算在上板完成)
        Chassis.Chassis_Follow_PID_Angle.Init(0.03f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f); //随动PID初始化

    #endif
}


#ifdef CHASSIS
#ifdef TRACK_LEG
/**
 * @brief 底盘给云台发送数据
 * 
 */
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    //发送数据给云台
    uint8_t robot_id,game_state;
    int16_t shoot_speed,Pos_X,Pos_Y;
    robot_id = Referee.Get_ID();
    game_state = Referee.Get_Game_Stage();
    shoot_speed = (int16_t)(Referee.Get_Shoot_Speed() * 1000.0f);
    Pos_X = (int16_t)(Referee.Get_Location_X() * 1000.0f);
    Pos_Y = (int16_t)(Referee.Get_Location_Y() * 1000.0f);
    memcpy(CAN3_Chassis_Tx_Gimbal_Data,&robot_id,sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 1,&game_state,sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 2, &shoot_speed, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 4, &Pos_X, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Gimbal_Data + 6, &Pos_Y, sizeof(int16_t));
}
#endif 
#ifdef AGV
float Class_Chariot::Get_Chassis_Coordinate_System_Angle_Rad()
{
    float GM6020_Angle_Rad = ((float)Motor_Yaw.Get_Now_Total_Encoder()) / 8191 * 2 * PI / 2;
    float Yaw_Angle_Rad = fabsf(fmodf(GM6020_Angle_Rad, 2.0f * PI));
    if(Motor_Yaw.Get_Now_Total_Round() < 0)
        Yaw_Angle_Rad = 2.0f * PI - Yaw_Angle_Rad;
    if(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= 2 * PI;//得到角度范围为[-PI,PI]
    
    Yaw_Angle_Rad -=  Reference_Angle;

    while(Yaw_Angle_Rad > PI)
        Yaw_Angle_Rad -= PI * 2.0f;
    while(Yaw_Angle_Rad < -PI)
        Yaw_Angle_Rad += PI * 2.0f;
    
    return (Yaw_Angle_Rad);
}

void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    //发送数据给云台
    uint8_t robot_id,game_state;
    int16_t shoot_speed,Pos_X,Pos_Y;
    robot_id = Referee.Get_ID();
    game_state = Referee.Get_Game_Stage();
    shoot_speed = (int16_t)(Referee.Get_Shoot_Speed() * 1000.0f);
    Pos_X = (int16_t)(Referee.Get_Location_X() * 1000.0f);
    Pos_Y = (int16_t)(Referee.Get_Location_Y() * 1000.0f);
    // Pos_X = (int16_t)(Uwb_pos_x * 1000.0f);
    // Pos_Y = (int16_t)(Uwb_pos_y * 1000.0f);
    memcpy(CAN2_Chassis_Tx_Gimbal_Data,&robot_id,sizeof(uint8_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 1,&game_state,sizeof(uint8_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &shoot_speed, sizeof(int16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 4, &Pos_X, sizeof(int16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 6, &Pos_Y, sizeof(int16_t));
}

void Class_Chariot::CAN_Chassis_Tx_Streeing_Wheel_Callback()
{
    //与舵小板通信有效数据为：角度、角速度
    uint16_t tmp_angle = 0;
    int16_t tmp_omega = 0;
    uint8_t chassis_status = (uint8_t)(Chassis.Get_Chassis_Control_Type());
    // 舵轮A 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[0].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[0].streeing_wheel_omega);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[0], &tmp_angle, 2);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[2], &tmp_omega, 2);
    //memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[4], &chassis_status, 1);
    // 舵轮B 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[1].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[1].streeing_wheel_omega);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[0], &tmp_angle, 2);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[2], &tmp_omega, 2);
    //memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[4], &chassis_status, 1);
    // 舵轮C 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[2].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[2].streeing_wheel_omega);
    // memcpy(&CAN1_0x1c_Tx_Streeing_Wheel_C_data[0], &tmp_angle, 2);
    // memcpy(&CAN1_0x1c_Tx_Streeing_Wheel_C_data[2], &tmp_omega, 2);
    // memcpy(&CAN1_0x1c_Tx_Streeing_Wheel_C_data[4], &chassis_status, 1);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[4], &tmp_angle, 2);
    memcpy(&CAN1_0x1b_Tx_Streeing_Wheel_B_data[6], &tmp_omega, 2);
    // 舵轮D 角度 与 角速度
    tmp_angle = Math_Float_To_Int(Chassis.wheel[3].streeing_wheel_angle, 0.0f, 8191.0f, 0, 0xFFFF);
    tmp_omega = (int16_t)(Chassis.wheel[3].streeing_wheel_omega);
    // memcpy(&CAN1_0x1d_Tx_Streeing_Wheel_D_data[0], &tmp_angle, 2);
    // memcpy(&CAN1_0x1d_Tx_Streeing_Wheel_D_data[2], &tmp_omega, 2);
    // memcpy(&CAN1_0x1d_Tx_Streeing_Wheel_D_data[4], &chassis_status, 1);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[4], &tmp_angle, 2);
    memcpy(&CAN1_0x1a_Tx_Streeing_Wheel_A_data[6], &tmp_omega, 2);

    memcpy(CAN1_0x01E_Tx_Data+5,&chassis_status,1);
		

}
#endif
#ifdef OLD
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Heat;
    uint16_t Cooling_Value;
    uint16_t Self_HP,Self_Outpost_HP,Oppo_Outpost_HP,Self_Base_HP,Ammo_number;
    uint8_t color,remaining_energy,supercap_proportion,radar_info,dart_target;
    uint16_t Pre_HP[6] = {0};
    uint16_t HP[6] = {0};
    uint8_t Flag[6] = {0};
    float Pre_Count[6] = {0};
    uint16_t Position[8] = {0};
    int16_t Bullet_Speed = 0.f;
    int16_t Self_Position_X,Self_Position_Y;
    int16_t Target_Position_X,Target_Position_Y;
    //数据更新
    if(Referee.Get_ID() == Referee_Data_Robots_ID_RED_SENTRY_7)
    {
        color = 1;
        Oppo_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_OUTPOST_11);
        Self_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_OUTPOST_11);
        Self_Base_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_BASE_10);

        for(int i = 0;i < 6;i++)
        {
            Pre_HP[i] = HP[i];
        }
        
        HP[0] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_HERO_1);
        HP[1] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_ENGINEER_2);
        HP[2] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_3);
        HP[3] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_4);
        HP[4] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_INFANTRY_5);
        HP[5] = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_SENTRY_7);

    }
    else if(Referee.Get_ID() == Referee_Data_Robots_ID_BLUE_SENTRY_7)
    {
        color = 0;
        Oppo_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_RED_OUTPOST_11);
        Self_Outpost_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_OUTPOST_11);
        Self_Base_HP = Referee.Get_HP(Referee_Data_Robots_ID_BLUE_BASE_10);

        for(int i = 0;i < 6;i++)
        {
            Pre_HP[i] = HP[i];
        }

        HP[0] = Referee.Get_HP(Referee_Data_Robots_ID_RED_HERO_1);
        HP[1] = Referee.Get_HP(Referee_Data_Robots_ID_RED_ENGINEER_2);
        HP[2] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_3);
        HP[3] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_4);
        HP[4] = Referee.Get_HP(Referee_Data_Robots_ID_RED_INFANTRY_5);
        HP[5] = Referee.Get_HP(Referee_Data_Robots_ID_RED_SENTRY_7);

    }
    Shooter_Heat = Referee.Get_Booster_17mm_1_Heat();
    if(Referee.Get_Shoot_Booster_Type() == Referee_Data_Robot_Booster_Type_BOOSTER_17MM_1)
    {
        Bullet_Speed = (int16_t)(Referee.Get_Shoot_Speed() * 100.f);
    }
    Self_HP = Referee.Get_HP();
    Ammo_number = Referee.Get_17mm_Remaining();
    Cooling_Value = Referee.Get_Booster_17mm_Heat_CD();
    remaining_energy = Referee.Get_Remaining_Energy();
    supercap_proportion = Chassis.Supercap.Get_Supercap_Proportion();
    Self_Position_X = (int16_t)(Referee.Get_Location_X() * 100.f);
    Self_Position_Y = (int16_t)(Referee.Get_Location_Y() * 100.f);
    Target_Position_X = (int16_t)(Referee.Get_Radar_Send_Coordinate_X() * 100.f);
    Target_Position_Y = (int16_t)(Referee.Get_Radar_Send_Coordinate_Y() * 100.f);
    radar_info = Referee.Get_Radar_Info();
    dart_target = Referee.Get_Dart_Command_Target() | (0x01 & Referee.Get_Sentry_Info_1() >> 19) << 2;

    for(int i = 0;i < 6;i++)//无敌状态辨认
    {
        if(HP[i] > 0 && Pre_HP[i] == 0)
        {
            Flag[i] = 1;
            Pre_Count[i] = DWT_GetTimeline_s();
        }
        if((DWT_GetTimeline_s() - Pre_Count[i]) > 7.f && Flag[i] == 1)
        {
            Flag[i] = 0;
            Pre_Count[i] = 0;
        }
    }

    Position[0] = Referee.Get_Hero_Position_X();
    Position[1] = Referee.Get_Hero_Position_Y();
    Position[2] = Referee.Get_Sentry_Position_X();
    Position[3] = Referee.Get_Sentry_Position_Y();
    Position[4] = Referee.Get_Infantry_3_Position_X();
    Position[5] = Referee.Get_Infantry_3_Position_Y();
    Position[6] = Referee.Get_Infantry_4_Position_X();
    Position[7] = Referee.Get_Infantry_4_Position_Y();

    //发送数据给云台
    //A包
    CAN3_Chassis_Tx_Data_A[0] = Referee.Get_Game_Stage();
    CAN3_Chassis_Tx_Data_A[1] = Referee.Get_Remaining_Time() >> 8;
    CAN3_Chassis_Tx_Data_A[2] = Referee.Get_Remaining_Time();
    CAN3_Chassis_Tx_Data_A[3] = Referee.Get_HP() >> 8;
    CAN3_Chassis_Tx_Data_A[4] = Referee.Get_HP();
    CAN3_Chassis_Tx_Data_A[5] = Self_Outpost_HP >> 8;
    CAN3_Chassis_Tx_Data_A[6] = Self_Outpost_HP;
    CAN3_Chassis_Tx_Data_A[7] = color << 7 | Flag[5] << 5 | Flag[4] << 4 | Flag[3] << 3 | Flag[2] << 2 | Flag[1] << 1 | Flag[0] << 0;

    //B包
    memcpy(CAN3_Chassis_Tx_Data_B + 0, &Self_Base_HP, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 2, &Oppo_Outpost_HP, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 4, &Ammo_number, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_B + 6, &Cooling_Value, sizeof(uint16_t));

    //C包
    memcpy(CAN3_Chassis_Tx_Data_C + 0, &Shooter_Heat, sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 4, &remaining_energy, sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 5, &supercap_proportion, sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 6, &radar_info, sizeof(uint8_t));
    memcpy(CAN3_Chassis_Tx_Data_C + 7, &dart_target, sizeof(uint8_t));

    //D包
    memcpy(CAN3_Chassis_Tx_Data_D + 0, &Position[0], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_D + 2, &Position[1], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_D + 4, &Position[2], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_D + 6, &Position[3], sizeof(uint16_t));

    //E包
    memcpy(CAN3_Chassis_Tx_Data_E + 0, &Self_Position_X, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Data_E + 2, &Self_Position_Y, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Data_E + 4, &Bullet_Speed, sizeof(int16_t));

    //F包
    memcpy(CAN3_Chassis_Tx_Data_F + 0, &Position[4], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_F + 2, &Position[5], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_F + 4, &Position[6], sizeof(uint16_t));
    memcpy(CAN3_Chassis_Tx_Data_F + 6, &Position[7], sizeof(uint16_t));

    //G包
    memcpy(CAN3_Chassis_Tx_Data_G + 0, &Target_Position_X, sizeof(int16_t));
    memcpy(CAN3_Chassis_Tx_Data_G + 2, &Target_Position_Y, sizeof(int16_t));
}
#endif
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
Struct_CAN_Referee_Rx_Data_t CAN_Referee_Rx_Data;
#ifdef CHASSIS
//控制类型字节
uint8_t control_type;
#ifdef TRACK_LEG
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    Gimbal_Alive_Flag++;
    // 控制类型字节
    uint8_t control_type,ui_type;
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 目标角速度
    //float chassis_omega;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    // 底盘和云台夹角（弧度制）
    float derta_angle;
    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y,tmp_gimbal_pitch;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    memcpy(&ui_type,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(uint8_t));
    memcpy(&tmp_gimbal_pitch, &CAN_Manage_Object->Rx_Buffer.Data[5], sizeof(uint16_t));
    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[7], sizeof(uint8_t));

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    if(fabs(gimbal_velocity_x) < 0.0002f)
        gimbal_velocity_x = 0.0f;
    if(fabs(gimbal_velocity_y) < 0.0002f)
        gimbal_velocity_y = 0.0f;

    float Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch, 0, 0x7FFF, -50.f, 50.f);
    Set_Gimbal_Pitch_Angle(Gimbal_Tx_Pitch_Angle);
    chassis_control_type = (Enum_Chassis_Control_Type)(control_type & 0x03);
    Chassis_Logics_Direction = (Enum_Chassis_Logics_Direction)(control_type >> 2 & 0x01);
    Yaw_Encoder_Control_Status = (Enum_Yaw_Encoder_Control_Status)(control_type >> 3 & 0x01);
    Fric_Status = (Enum_Fric_Status)(control_type >> 4 & 0x01);
    Supercap_Control_Status = (Enum_Supercap_Control_Status)(control_type >> 5 & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)(control_type >> 6 & 0x01);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)(control_type >> 7 & 0x01);
    //UI_Radar_Target = (Enum_Radar_Target)(ui_type & 0x01);
    //UI_Radar_Target_Pos = (Enum_Radar_Target_Outpost)(ui_type >> 1 & 0x03);
    //UI_Radar_Control_Type = (Enum_Radar_Control_Type)(ui_type >> 3 & 0x01);
    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);
    //小陀螺补偿角度
    if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Positive||
        Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Negative)
    {
        Offset_Angle = 15.0f * DEG_TO_RAD;
    }
    else
    {
        Offset_Angle = 0.0f;
    }
    
    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    //Chassis_Angle = Chassis_Coordinate_System_Angle_Rad;
    derta_angle = (Reference_Angle-Reference_Angle) - Chassis_Angle + Offset_Angle;
    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));
    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
}

void Class_Chariot::Control_Chassis_Omega_TIM_PeriodElapsedCallback()
{
	
    // 目标角速度
    float chassis_omega;

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
        chassis_omega = Chassis.Get_Spin_Omega();
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Negative)
        chassis_omega = -Chassis.Get_Spin_Omega();

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {   
        PID_Chassis_Fllow.Set_Target(Reference_Angle-Reference_Angle);
        PID_Chassis_Fllow.Set_Now(Chassis_Angle);
        PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        chassis_omega = -PID_Chassis_Fllow.Get_Out();
    }

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        chassis_omega = 0;
    }

    Chassis.Set_Target_Omega(chassis_omega);
}
#endif
#ifdef AGV
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{
    Gimbal_Alive_Flag++;
    // 控制类型字节
    uint8_t control_type,ui_type;
    // 云台坐标系的目标速度
    float gimbal_velocity_x, gimbal_velocity_y;
    // 底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    // 目标角速度
    //float chassis_omega;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    // 底盘和云台夹角（弧度制）
    float derta_angle;
    // float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y,tmp_gimbal_pitch;

    memcpy(&tmp_velocity_x, &CAN_Manage_Object->Rx_Buffer.Data[0], sizeof(uint16_t));
    memcpy(&tmp_velocity_y, &CAN_Manage_Object->Rx_Buffer.Data[2], sizeof(uint16_t));
    memcpy(&ui_type,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(uint8_t));
    memcpy(&tmp_gimbal_pitch, &CAN_Manage_Object->Rx_Buffer.Data[5], sizeof(uint16_t));
    memcpy(&control_type, &CAN_Manage_Object->Rx_Buffer.Data[7], sizeof(uint8_t));

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    if(fabs(gimbal_velocity_x) < 0.0002f)
        gimbal_velocity_x = 0.0f;
    if(fabs(gimbal_velocity_y) < 0.0002f)
        gimbal_velocity_y = 0.0f;

    float Gimbal_Tx_Pitch_Angle = Math_Int_To_Float(tmp_gimbal_pitch, 0, 0x7FFF, -50.f, 50.f);
    Set_Gimbal_Pitch_Angle(Gimbal_Tx_Pitch_Angle);
    chassis_control_type = (Enum_Chassis_Control_Type)(control_type & 0x03);
    Chassis_Logics_Direction = (Enum_Chassis_Logics_Direction)(control_type >> 2 & 0x01);
    Yaw_Encoder_Control_Status = (Enum_Yaw_Encoder_Control_Status)(control_type >> 3 & 0x01);
    Fric_Status = (Enum_Fric_Status)(control_type >> 4 & 0x01);
    Supercap_Control_Status = (Enum_Supercap_Control_Status)(control_type >> 5 & 0x01);
    MiniPC_Status = (Enum_MiniPC_Status)(control_type >> 6 & 0x01);
    Referee_UI_Refresh_Status = (Enum_Referee_UI_Refresh_Status)(control_type >> 7 & 0x01);
    //UI_Radar_Target = (Enum_Radar_Target)(ui_type & 0x01);
    //UI_Radar_Target_Pos = (Enum_Radar_Target_Outpost)(ui_type >> 1 & 0x03);
    //UI_Radar_Control_Type = (Enum_Radar_Control_Type)(ui_type >> 3 & 0x01);
    // 设定底盘控制类型
    Chassis.Set_Chassis_Control_Type(chassis_control_type);
    //小陀螺补偿角度
    if(Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Positive||
        Chassis.Get_Chassis_Control_Type()==Chassis_Control_Type_SPIN_Negative)
    {
        Offset_Angle = 15.0f * DEG_TO_RAD;
    }
    else
    {
        Offset_Angle = 0.0f;
    }
    
    // 获取云台坐标系和底盘坐标系的夹角（弧度制）
    Chassis_Angle = Chassis_Coordinate_System_Angle_Rad;
    derta_angle = (Reference_Angle-Reference_Angle) - Chassis_Angle + Offset_Angle;
    // 云台坐标系的目标速度转为底盘坐标系的目标速度
    chassis_velocity_x = (float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle));
    chassis_velocity_y = (float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle));
    // 设定底盘目标速度
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
}

void Class_Chariot::Control_Chassis_Omega_TIM_PeriodElapsedCallback()
{
	
    // 目标角速度
    float chassis_omega;

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
        chassis_omega = Chassis.Get_Spin_Omega();
    else if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Negative)
        chassis_omega = -Chassis.Get_Spin_Omega();

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
    {   
        PID_Chassis_Fllow.Set_Target(Reference_Angle-Reference_Angle);
        PID_Chassis_Fllow.Set_Now(Chassis_Angle);
        PID_Chassis_Fllow.TIM_Adjust_PeriodElapsedCallback();
        chassis_omega = -PID_Chassis_Fllow.Get_Out();
    }

    if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_DISABLE)
    {
        chassis_omega = 0;
    }

    Chassis.Set_Target_Omega(chassis_omega);
}
#endif
#ifdef OLD
void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback()
{   
    Gimbal_Alive_Flag++;
    //底盘坐标系的目标速度
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //超电控制类型
    Enum_Supercap_Mode supercap_mode;
    //float映射到int16之后的速度
    int16_t tmp_velocity_x, tmp_velocity_y, tmp_omega;
    switch(CAN_Manage_Object->Rx_Buffer.Header.Identifier){
        case (0x77):
        {
            memcpy(&tmp_velocity_x,&CAN_Manage_Object->Rx_Buffer.Data[0],sizeof(int16_t));
            memcpy(&tmp_velocity_y,&CAN_Manage_Object->Rx_Buffer.Data[2],sizeof(int16_t));
            memcpy(&tmp_omega,&CAN_Manage_Object->Rx_Buffer.Data[4],sizeof(int16_t));
            memcpy(&supercap_mode,&CAN_Manage_Object->Rx_Buffer.Data[6],sizeof(uint8_t));
            memcpy(&control_type,&CAN_Manage_Object->Rx_Buffer.Data[7],sizeof(uint8_t));
            
            #ifdef OLD
            chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x,-450,450,-4,4);
            chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y,-450,450,-4,4);
            chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -4.f, 4.f)/ Chassis_Radius;//映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度
            #else
            chassis_velocity_x = Math_Int_To_Float(tmp_velocity_x,-450,450,-20.f,20.f);
            chassis_velocity_y = Math_Int_To_Float(tmp_velocity_y,-450,450,-20.f,20.f);
            chassis_omega = Math_Int_To_Float(tmp_omega, -200, 200, -80.f, 80.f);
            #endif
            chassis_control_type = (Enum_Chassis_Control_Type)control_type;
            //设定底盘控制类型
            Chassis.Set_Chassis_Control_Type(chassis_control_type);
            if(chassis_omega < 0.5f && chassis_omega > -0.5f)chassis_omega = 0;
            //设定底盘目标速度
            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
            #ifdef OMNI_WHEEL
                Chassis.Set_Target_Velocity_X(-chassis_velocity_x);
            #endif
            Chassis.Set_Target_Omega(chassis_omega);//线速度
            //Chassis.Set_Supercap_Mode(supercap_mode);
            break;
        }
        case (0x95):
        {
            memcpy(&CAN_Referee_Rx_Data,&CAN_Manage_Object->Rx_Buffer.Data,sizeof(Struct_CAN_Referee_Rx_Data_t));
            break;
        }
    }


}
#endif
#endif

/**
 * @brief can回调函数处理底盘发来的数据
 *
 */
Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
Referee_Rx_A_t PRE_CAN3_Chassis_Rx_Data_A;
Referee_Rx_B_t CAN3_Chassis_Rx_Data_B;
Referee_Rx_C_t CAN3_Chassis_Rx_Data_C;
Referee_Rx_D_t CAN3_Chassis_Rx_Data_D;
Referee_Rx_E_t CAN3_Chassis_Rx_Data_E;
Referee_Rx_F_t CAN3_Chassis_Rx_Data_F;
Referee_Rx_G_t CAN3_Chassis_Rx_Data_G;
float speed_a,speed_b;
#ifdef GIMBAL
#ifdef TRACK_LEG
Enum_Referee_Data_Robots_ID robo_id;
Enum_Referee_Game_Status_Stage game_stage;
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;
    float shoot_speed;
    memcpy(&robo_id,CAN_Manage_Object->Rx_Buffer.Data,sizeof(uint8_t));
    memcpy(&game_stage,CAN_Manage_Object->Rx_Buffer.Data+1,sizeof(uint8_t));
    memcpy(&shoot_speed,CAN_Manage_Object->Rx_Buffer.Data+2,sizeof(float));
    Referee.Set_Robot_ID(robo_id);
    Referee.Set_Game_Stage(game_stage);
    Referee.Set_Shoot_Speed(shoot_speed);
}
#endif
#ifdef OLD
void Class_Chariot::CAN_Gimbal_Rx_Chassis_Callback()
{
    Chassis_Alive_Flag++;
    switch(CAN_Manage_Object->Rx_Buffer.Header.Identifier){
        case (0x188):{
            memcpy(&PRE_CAN3_Chassis_Rx_Data_A, &CAN3_Chassis_Rx_Data_A, sizeof(Referee_Rx_A_t));
            CAN3_Chassis_Rx_Data_A.game_process = CAN_Manage_Object->Rx_Buffer.Data[0];
            CAN3_Chassis_Rx_Data_A.remaining_time = CAN_Manage_Object->Rx_Buffer.Data[1] << 8 | CAN_Manage_Object->Rx_Buffer.Data[2];
            CAN3_Chassis_Rx_Data_A.self_blood = CAN_Manage_Object->Rx_Buffer.Data[3] << 8 | CAN_Manage_Object->Rx_Buffer.Data[4];
            CAN3_Chassis_Rx_Data_A.self_outpost_HP = CAN_Manage_Object->Rx_Buffer.Data[5] << 8 | CAN_Manage_Object->Rx_Buffer.Data[6];
            CAN3_Chassis_Rx_Data_A.color_invincible_state = CAN_Manage_Object->Rx_Buffer.Data[7];
            break;
        }
        case (0x199):{
            memcpy(&CAN3_Chassis_Rx_Data_B, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_B_t));
            break;
        }
        case (0x178):{
            memcpy(&CAN3_Chassis_Rx_Data_C, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_C_t));
            Booster.Set_Heat(CAN3_Chassis_Rx_Data_C.Booster_Heat);
            break;
        }
        case (0x198):{
            memcpy(&CAN3_Chassis_Rx_Data_D, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_D_t));
            break;
        }
        case (0x197):{
            memcpy(&CAN3_Chassis_Rx_Data_E, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_E_t));
            speed_a = (float)(CAN3_Chassis_Rx_Data_E.Bullet_Speed / 100.f);
            break;
        }
        case (0x196):{
            memcpy(&CAN3_Chassis_Rx_Data_F, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_F_t));
            break;
        }
        case (0x191):{
            memcpy(&CAN3_Chassis_Rx_Data_G, CAN_Manage_Object->Rx_Buffer.Data, sizeof(Referee_Rx_G_t));
            break;
        }
    }
}
#endif
#endif


/**
 * @brief can回调函数给地盘发送数据
 *
 */
#ifdef GIMBAL
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    uint8_t control_type,ui_type;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0, gimbal_pitch;
    // 映射之后的目标速度 int16_t
    uint16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0,tmp_gimbal_yaw = 0;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    
    // 控制类型字节
    //MiniPC_Status =(Enum_MiniPC_Status)MiniPC.Get_Radar_Enable_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    //gimbal_pitch = Gimbal.Motor_Pitch.Get_True_Angle_Pitch();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    control_type = (uint8_t)(Referee_UI_Refresh_Status << 7 |  MiniPC_Status<< 6 | Supercap_Control_Status << 5 | Fric_Status << 4 | Yaw_Encoder_Control_Status << 3 | Chassis_Logics_Direction << 2 | chassis_control_type);

    // 设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max(), 0, 0x7FFF);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(uint16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max(), 0, 0x7FFF);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(uint16_t));

    //ui_type = (uint8_t)(UI_Radar_Control_Type << 3 | UI_Radar_Target_Pos << 1 | UI_Radar_Target);
    //memcpy(CAN3_Gimbal_Tx_Chassis_Data + 4, &ui_type, 1);

    //tmp_gimbal_yaw = Math_Float_To_Int(gimbal_pitch, -50.f, 50.f ,0,0x7FFF);
    //memcpy(CAN3_Gimbal_Tx_Chassis_Data + 5, &tmp_gimbal_yaw, sizeof(uint16_t));

    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 7, &control_type, sizeof(uint8_t));
}
#endif
/**
 * @brief 底盘控制逻辑
 *
 */  		
float Offset_K = 0.175f;
#ifdef GIMBAL
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y, dr16_lr_yaw;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_omega = 0;
    if (Get_DR16_Control_Type() == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);   
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Negative);
            }
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下 位姿切换
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);

        }
        Chassis.Set_Target_Velocity_X(chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
        Chassis.Set_Target_Omega(chassis_omega);
    }
#ifdef OLD
    //遥控器摇杆值
    float dr16_l_x, dr16_l_y;    
    //云台坐标系速度目标值 float
    float gimbal_velocity_x = 0, gimbal_velocity_y = 0;      
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_omega = 0;  
    //云台坐标系角度目标值 float
    float gimbal_angle = 0,chassis_angle = 0,relative_angle = 0;
	
    //排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

    //设定矩形到圆形映射进行控制
    gimbal_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max() ;
    gimbal_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max() ;

    //遥控器操作逻辑
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    // volatile int DR16_Right_Switch_Status = DR16.Get_Right_Switch();
    switch(DR16_Left_Switch_Status){
        case (DR16_Switch_Status_UP):   // 左上 小陀螺模式
        {

            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            Chassis.Set_Pose_Control_Type(Pose_DISABLE);
            break;
        }
        case(DR16_Switch_Status_MIDDLE): // 左中 随动模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            break;
        }
    }

    Chassis.Set_Target_Velocity_X(gimbal_velocity_y);
    Chassis.Set_Target_Velocity_Y(-gimbal_velocity_x);//前x左y正

    //相对角度计算
    gimbal_angle = Gimbal.Motor_Yaw.Get_Zero_Position();
    chassis_angle = addSampleAndFilter(Gimbal.Motor_Yaw.Get_Now_Angle(),5);
    relative_angle = chassis_angle - gimbal_angle ;
    
    MiniPC.Set_Gimbal_Now_Relative_Angle(relative_angle);

    relative_angle = DEG_TO_RAD * relative_angle;

    if(MiniPC.Get_MiniPC_Status() != MiniPC_Status_DISABLE && DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN){//上位机导航信息接收
        if(MiniPC.Get_Chassis_Target_Velocity_X() != 0 || MiniPC.Get_Chassis_Target_Velocity_Y() != 0){
            Chassis.Set_Target_Velocity_X(float(MiniPC.Get_Chassis_Target_Velocity_X() / 100.f));
            Chassis.Set_Target_Velocity_Y(float(MiniPC.Get_Chassis_Target_Velocity_Y() / 100.f));
        }
    }
   
    //云台到底盘坐标系转换
    volatile int Chassis_control_type = Chassis.Get_Chassis_Control_Type(); 
     switch(Chassis_control_type){
        case(Chassis_Control_Type_DISABLE):{//失能
            chassis_velocity_x = 0;
            chassis_velocity_y = 0;
            chassis_omega = 0;
            break;
        }
        case(Chassis_Control_Type_FLLOW):
        {   //随动 附有非随动和受击陀螺逻辑
            if(Gimbal.Motor_Yaw.Get_DJI_Motor_Status() == LK_Motor_Status_DISABLE){//大yaw离线失能
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);           
            }
            else{//正常随动
                Chassis.Chassis_Follow_PID_Angle.Set_Target(0);
                Chassis.Chassis_Follow_PID_Angle.Set_Now(relative_angle * 180 / PI);
                Chassis.Chassis_Follow_PID_Angle.TIM_Adjust_PeriodElapsedCallback();
                chassis_omega = Chassis.Chassis_Follow_PID_Angle.Get_Out() / 2;
                chassis_velocity_x = Chassis.Get_Target_Velocity_X() * cos(relative_angle) - Chassis.Get_Target_Velocity_Y() * sin(relative_angle);
                chassis_velocity_y = Chassis.Get_Target_Velocity_X() * sin(relative_angle) + Chassis.Get_Target_Velocity_Y() * cos(relative_angle);
            }
            break;
        }
        case(Chassis_Control_Type_SPIN):
        {
            chassis_omega = 0.75f;
            relative_angle += Gimbal.Motor_Yaw.Get_Now_Omega_Radian() * Offset_K;
            chassis_velocity_x = Chassis.Get_Target_Velocity_X() * cos(relative_angle) - Chassis.Get_Target_Velocity_Y() * sin(relative_angle);
            chassis_velocity_y = Chassis.Get_Target_Velocity_X() * sin(relative_angle) + Chassis.Get_Target_Velocity_Y() * cos(relative_angle);
            if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN &&
                DR16.Get_Left_Switch() == DR16_Switch_Status_UP)
            {
                chassis_omega = -0.75f;
            }
            break;
        }
        
    }
    if(chassis_omega > 4)chassis_omega = 4;
    if(chassis_omega < -4)chassis_omega = -4;
    
    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);//前x左y正
    Chassis.Set_Target_Omega(chassis_omega);
#endif    
}
#endif
#ifdef CHASSIS
#ifdef Only_Chassis
void Class_Chariot::Control_Chassis()
{
    // 遥控器摇杆值
    float dr16_l_x, dr16_l_y, dr16_r_x, dr16_f_yaw;
    // 云台坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    // 目标角速度
    float chassis_omega = 0;
    // 底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    if (DR16_Control_Type == DR16_Control_Type_REMOTE)
    {
        // 排除遥控器死区
        dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
        dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;
        dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;

        // 设定矩形到圆形映射进行控制
        chassis_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
        chassis_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);   
            chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Positive);
            chassis_omega = Chassis.Get_Spin_Omega();
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_Negative);
                chassis_omega = -Chassis.Get_Spin_Omega();
            }
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) //左下 位姿切换
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
            #ifdef LOCKED_SWITCH
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN)
            {
                Chassis.Set_Pose_Control_Type(Pose_STANDBY);
            }
            else if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE)
            {
                Chassis.Set_Pose_Control_Type(Pose_ENABLE);
            }
            #endif
            #ifdef AUTO_SWITCH
            if(DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN)
            {
                //开启履带驱动电机
                Chassis.Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Chassis.Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Chassis.Motor_Track[0].Set_Target_Omega_Radian(5.0f);
                Chassis.Motor_Track[1].Set_Target_Omega_Radian(5.0f);
            }
            else
            {
                Chassis.Motor_Track[0].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Chassis.Motor_Track[1].Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
                Chassis.Motor_Track[0].Set_Target_Omega_Radian(0.0f);
                Chassis.Motor_Track[1].Set_Target_Omega_Radian(0.0f);
            }
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE)
            {
                Chassis.Set_Pose_Control_Type(Pose_STANDBY);//缩腿
            }
            #endif
        }
        Chassis.Set_Target_Velocity_X(chassis_velocity_x);
        Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
        Chassis.Set_Target_Omega(chassis_omega);
        //力控底盘任务
        chassis_control_type = Chassis.Get_Chassis_Control_Type();
        if(chassis_control_type == Chassis_Control_Type_DISABLE)
        {
            Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);
        }
        else
        {
            Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_NORMAL__);
            chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();
            if(chassis_control_type == Chassis_Control_Type_SPIN_Positive)
            {
                chassis_omega = Chassis_Spin_Omega;
            }
            else if(chassis_control_type == Chassis_Control_Type_SPIN_Negative)
            {
                chassis_omega = -Chassis_Spin_Omega;
            }
        }
        Force_Control_Chassis.Set_Target_Velocity_X(chassis_velocity_y);
        Force_Control_Chassis.Set_Target_Velocity_Y(-chassis_velocity_x);
        Force_Control_Chassis.Set_Target_Omega(chassis_omega);
    }
}
#endif
#endif

/**
 * @brief 鼠标数据转换
 *
 */
#ifdef GIMBAL
void Class_Chariot::Transform_Mouse_Axis(){
        True_Mouse_X = -DR16.Get_Mouse_X();
        True_Mouse_Y =  DR16.Get_Mouse_Y();
        True_Mouse_Z =  DR16.Get_Mouse_Z();
}
#endif
/**
 * @brief 云台控制逻辑
 *
 */
#ifdef GIMBAL
void Class_Chariot::Control_Gimbal()
{
    // 角度目标值
    float tmp_gimbal_yaw, tmp_gimbal_pitch;
    // 遥控器摇杆值
    float dr16_y, dr16_r_y;

    // 排除遥控器死区
    dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
    dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;

    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();
    tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_Target_Angle();

    // 遥控器操作逻辑
    tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution;
    tmp_gimbal_pitch -= dr16_r_y * DR16_Pitch_Angle_Resolution;
    // 限制角度范围 处理yaw轴180度问题
    if ((tmp_gimbal_yaw ) > 180.0f)
    {
        tmp_gimbal_yaw -= (360.0f);
    }
    else if ((tmp_gimbal_yaw) < -180.0f)
    {
        tmp_gimbal_yaw += (360.0f);
    }

    if(tmp_gimbal_pitch > 18.0f)tmp_gimbal_pitch = 18.0f;
    if(tmp_gimbal_pitch < -25.0f)tmp_gimbal_pitch = -25.0f;

    if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下 位姿切换
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
    }
    else // 其余位置都是遥控器控制
    {
        // 中间遥控模式
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);

        // 设定角度
        Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
        Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);
    }

    if(Gimbal.Get_Gimbal_Control_Type()==Gimbal_Control_Type_NORMAL)
    {
        if(DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_MIDDLE_DOWN)
        {
        Gimbal.Joint_Test.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Gimbal.Joint_Test.Set_Target_Angle(PI);
        Gimbal.Joint_Test.Set_Target_Omega(PI);
        // Gimbal.Joint_Test.Set_Target_Torque(0.0f);
        // Gimbal.Joint_Test.Set_MIT_K_P(6.0f);
        // Gimbal.Joint_Test.Set_MIT_K_D(0.2f);
        }
        else if(DR16.Get_Right_Switch() == DR16_Switch_Status_TRIG_DOWN_MIDDLE)
        {
        Gimbal.Joint_Test.Set_DM_Control_Status(DM_Motor_Control_Status_ENABLE);
        Gimbal.Joint_Test.Set_Target_Angle(0.0f);
        Gimbal.Joint_Test.Set_Target_Omega(PI);
        // Gimbal.Joint_Test.Set_Target_Torque(0.0f);
        // Gimbal.Joint_Test.Set_MIT_K_P(6.0f);
        // Gimbal.Joint_Test.Set_MIT_K_D(0.2f);
        }
    }
}
#endif
/**
 * @brief 发射机构控制逻辑
 *
 */
int Booster_Sign = 0;
#ifdef GIMBAL
void Class_Chariot::Control_Booster()
{
    static uint8_t booster_sign = 0;
    volatile int DR16_Left_Switch_Status = DR16.Get_Left_Switch();
    volatile int DR16_Right_Switch_Status = DR16.Get_Right_Switch();
    switch (DR16_Right_Switch_Status){
        case(DR16_Switch_Status_UP): // 右上 开启摩擦轮和发射机构
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            if (DR16.Get_Yaw() < 0.2 && DR16.Get_Yaw() > -0.2)
            {
                booster_sign = 0;
            }
            else if (DR16.Get_Yaw() > 0.8 && booster_sign == 0)
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                booster_sign = 1;
            }
            break;    
        }
        case(DR16_Switch_Status_MIDDLE):
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            break;
        }
        case(DR16_Switch_Status_DOWN):
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            break;
        }
    }


}
#endif

/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
    #ifdef CHASSIS
    #ifdef AGV
        //获取底盘坐标系角度
        Chassis_Coordinate_System_Angle_Rad = Get_Chassis_Coordinate_System_Angle_Rad();
        // 底盘给云台发消息
        CAN_Chassis_Tx_Gimbal_Callback();
        //底盘给分别给四个舵轮发消息
        CAN_Chassis_Tx_Streeing_Wheel_Callback();
        //底盘给舵小板发送最大功率
        //超电使用策略
        if(Supercap_Control_Status == Supercap_Control_Status_ENABLE)
        {
            Chassis.Supercap.Set_Supercap_Usage_Stratage(Supercap_Usage_Stratage_Supercap_BufferPower);
        }
        else
        {
            Chassis.Supercap.Set_Supercap_Usage_Stratage(Supercap_Usage_Stratage_Referee_BufferPower);
        }
        Chassis.Supercap.TIM_Supercap_PeriodElapsedCallback();
        //底盘Omega控制
        Control_Chassis_Omega_TIM_PeriodElapsedCallback();
        //底盘解算控制
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        //画UI
        //UI_Flying_Risk_Status = (uint8_t)(Referee.Get_Referee_Data_Interaction_Students() & 0x01);
        //UI_DogHole_1_Risk_Status = (uint8_t)((Referee.Get_Referee_Data_Interaction_Students() >> 1) & 0x01);
        //UI_Steps_Risk_Status = (uint8_t)((Referee.Get_Referee_Data_Interaction_Students() >> 2) & 0x01);
        //UI_DogHole_2_Risk_Status = (uint8_t)((Referee.Get_Referee_Data_Interaction_Students() >> 3) & 0x01);
        static uint8_t mod20 = 0;
        mod20++;
        if(mod20==20)
        {
            //Chariot_Referee_UI_Tx_Callback(Referee_UI_Refresh_Status);
            mod20 = 0;
        };
	#endif		
    #ifdef TRACK_LEG
        // 底盘给云台发消息
        CAN_Chassis_Tx_Gimbal_Callback();
        // //底盘Omega控制
        // Control_Chassis_Omega_TIM_PeriodElapsedCallback();
        //底盘解算控制
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        static uint8_t mod2 = 0;
        mod2++;
        if (mod2 == 2)
        {
            //补充力控底盘
            Force_Control_Chassis.TIM_2ms_Control_PeriodElapsedCallback();
            Force_Control_Chassis.TIM_2ms_Resolution_PeriodElapsedCallback();
            mod2 = 0;
        }
    #endif	
    #elif defined(GIMBAL)

        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster.TIM_Calculate_PeriodElapsedCallback();
        // Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);
        //传输数据给上位机
        MiniPC.TIM_Write_PeriodElapsedCallback();
        //给下板发送数据
        CAN_Gimbal_Tx_Chassis_Callback();
    #endif   
}

/**
 * @brief 判断DR16控制数据来源
 *
 */
#ifdef GIMBAL
void Class_Chariot::Judge_DR16_Control_Type()
{
    DR16_Control_Type = DR16_Control_Type_REMOTE;
}
#endif
#ifdef CHASSIS
void Class_Chariot::Judge_DR16_Control_Type()
{
    DR16_Control_Type = DR16_Control_Type_REMOTE;
}
#endif
/**
 * @brief 控制回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM_Control_Callback()
{
    //判断DR16控制数据来源
    Judge_DR16_Control_Type();

    //底盘，云台，发射机构控制逻辑
    Control_Chassis();
    Control_Gimbal();
    Control_Booster();
}
#endif
#ifdef CHASSIS

void Class_Chariot::TIM_Control_Callback()
{
    //判断DR16控制数据来源
    Judge_DR16_Control_Type();

    //底盘，云台，发射机构控制逻辑
    Control_Chassis();
}
#endif
/**
 * @brief 在线判断回调函数
 *
 */
extern Referee_Rx_A_t CAN3_Chassis_Rx_Data_A;
void Class_Chariot::TIM1msMod50_Alive_PeriodElapsedCallback()
{
    static uint8_t mod50 = 0;
    static uint8_t mod50_mod3 = 0;
    mod50++;
    if (mod50 == 50)
    {
        mod50_mod3++;
        //TIM_Unline_Protect_PeriodElapsedCallback();
        #ifdef CHASSIS
            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
            Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            #ifndef AGV
            for (auto& wheel : Chassis.Motor_Wheel) {
                wheel.TIM_Alive_PeriodElapsedCallback();
            }     
            #endif
            #ifdef TRACK_LEG
            Chassis.Motor_Joint[0].TIM_Alive_PeriodElapsedCallback();
            Chassis.Motor_Joint[1].TIM_Alive_PeriodElapsedCallback();
            Chassis.Motor_Track[0].TIM_Alive_PeriodElapsedCallback();
            Chassis.Motor_Track[1].TIM_Alive_PeriodElapsedCallback();
            //力控底盘
            Force_Control_Chassis.TIM_100ms_Alive_PeriodElapsedCallback();
            #endif
            if(mod50_mod3%3 == 0)
            {
                TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;
            }  
            #ifdef Only_Chassis
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();	
            //Force_Control_Chassis.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();
            Chassis.BoardDM_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();
            #ifdef defined(USE_DR16)
                #ifdef DEBUG
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #endif
            #endif
            #endif
        #elif defined(GIMBAL)

            if(mod50_mod3%3==0)
            {
                //判断底盘通讯在线状态
                TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback();    
                DR16.TIM1msMod50_Alive_PeriodElapsedCallback();	   
                mod50_mod3 = 0;         
            }
            #ifdef defined(USE_DR16)
                #ifdef DEBUG
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #else
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
                {
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                }
                #endif
            #elif defined(USE_VT13)
                #ifdef DEBUG
                    if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #else
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
                {
                    if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                }
                #endif

            #endif
                
            Gimbal.Motor_Pitch.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Motor_Yaw_DM4310.TIM_Alive_PeriodElapsedCallback();
            Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();
            Gimbal.Joint_Test.TIM_Alive_PeriodElapsedCallback();

            Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            #ifdef Single_Friction
            Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
			#endif
            #ifdef Double_Friction
            for (auto i = 0; i < 4; i++)
            {
                Booster.Fric[i].TIM_Alive_PeriodElapsedCallback();
            }
            #endif
			MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();
            Referee.TIM1msMod50_Alive_PeriodElapsedCallback();

        #endif

        mod50 = 0;
    }    
}

/**
 * @brief 离线保护函数
 *
 */
void Class_Chariot::TIM_Unline_Protect_PeriodElapsedCallback()
{
    //云台离线保护
    #ifdef GIMBAL
        #ifdef defined(USE_DR16)
                #ifdef DEBUG
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #else
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
                {
                    if (DR16.Get_DR16_Status() == DR16_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                }
                #endif
            #elif defined(USE_VT13)
                #ifdef DEBUG
                    if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                #else
                if(CAN3_Chassis_Rx_Data_A.game_process != 4)
                {
                    if (VT13.Get_VT13_Status() == VT13_Status_DISABLE)
                    {
                        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
                        Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
                        Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
                    }
                }
                #endif

        #endif

    #endif

    //底盘离线保护
    #ifdef CHASSIS
        
    #endif

}

/**
 * @brief 底盘通讯在线判断回调函数
 *
 */
#ifdef GIMBAL
void Class_Chariot::TIM1msMod50_Chassis_Communicate_Alive_PeriodElapsedCallback()
{
    if (Chassis_Alive_Flag == Pre_Chassis_Alive_Flag)
    {
        Chassis_Status = Chassis_Status_DISABLE;
    }
    else
    {
        Chassis_Status = Chassis_Status_ENABLE;
    }
    Pre_Chassis_Alive_Flag = Chassis_Alive_Flag;   
}
#endif

#ifdef CHASSIS
void Class_Chariot::TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback()
{
    if (Gimbal_Alive_Flag == Pre_Gimbal_Alive_Flag)
    {
        Gimbal_Status = Gimbal_Status_DISABLE;
    }
    else
    {
        Gimbal_Status = Gimbal_Status_ENABLE;
    }
    Pre_Gimbal_Alive_Flag = Gimbal_Alive_Flag;  
}
#endif
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart5); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif
#ifdef CHASSIS
#ifdef Only_Chassis
void Class_FSM_Alive_Control::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);
            Chariot->Force_Control_Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE__);

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart5.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart5); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart5, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif
#endif
/**
 * @brief 机器人遥控器离线控制状态转移函数
 *
 */
#ifdef GIMBAL
void Class_FSM_Alive_Control_VT13::Reload_TIM_Status_PeriodElapsedCallback(){
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
        // 离线检测状态
        case (0):
        {
            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart9.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }

            //转移为 在线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {             
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            //超过一秒的遥控器离线 跳转到 遥控器关闭状态
            if(Status[Now_Status_Serial].Time > 1000)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(1);
            }
        }
        break;
        // 遥控器关闭状态
        case (1):
        {
            //离线保护
            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
                Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
                Status[Now_Status_Serial].Time = 0;
                Set_Status(2);
            }

            // 遥控器中途断联导致错误离线 跳转到 遥控器串口错误状态
            if (huart9.ErrorCode)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(4);
            }
            
        }
        break;
        // 遥控器在线状态
        case (2):
        {
            //转移为 刚离线状态
            if(Chariot->VT13.Get_VT13_Status() == VT13_Status_DISABLE)
            {
                Status[Now_Status_Serial].Time = 0;
                Set_Status(3);
            }
        }
        break;
        //刚离线状态
        case (3):
        {
            //记录离线检测前控制模式
            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

            //无条件转移到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
        //遥控器串口错误状态
        case (4):
        {
            HAL_UART_DMAStop(&huart9); // 停止以重启
            //HAL_Delay(10); // 等待错误结束
            HAL_UARTEx_ReceiveToIdle_DMA(&huart9, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);

            //处理完直接跳转到 离线检测状态
            Status[Now_Status_Serial].Time = 0;
            Set_Status(0);
        }
        break;
    } 
}
#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
