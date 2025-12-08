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

        //底盘
        Chassis.Referee = &Referee;
        Chassis.Init();
        PID_Chassis_Follow.Init(-0.09f, 0.0f, -0.022f, 0.0f, 4.0f, 4.0f);
				Motor_Yaw.Init(&hfdcan2, DM_Motor_ID_0xA3, DM_Motor_Control_Method_MIT_IMU_Angle, 0,20.94f,2.0f);
        
        //超电
        Chassis.Supercap.Referee = &Referee;

        Chassis.Set_Velocity_X_Max(4.0f);
        Chassis.Set_Velocity_Y_Max(4.0f);
	
        #ifdef TEST

        //遥控器
        #ifdef USE_DR16
        //遥控器离线控制 状态机
        FSM_Alive_Control.Chariot = this;
        FSM_Alive_Control.Init(5, 0);

        //遥控器
        DR16.Init(&huart5,&huart1);
        DR16_Dead_Zone = __DR16_Dead_Zone;  
        #endif

        #endif

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
				
        //上位机
        MiniPC.Init(&MiniPC_USB_Manage_Object,&UART8_Manage_Object,&CAN3_Manage_Object);
        MiniPC.IMU = &Gimbal.Boardc_BMI;
        MiniPC.Referee = &Referee;

        //底盘随动环pid初始化(角度结算在上板完成)
        Chassis.Chassis_Follow_PID_Angle.Init(0.03f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f); //随动PID初始化

    #endif
}


#ifdef CHASSIS
void Class_Chariot::CAN_Chassis_Tx_Gimbal_Callback()
{
    uint16_t Shooter_Barrel_Heat;
    uint16_t Shooter_Barrel_Heat_Limit;
    uint16_t Shooter_Speed;
    Shooter_Barrel_Heat_Limit = Referee.Get_Booster_17mm_1_Heat_Max();
    Shooter_Barrel_Heat = Referee.Get_Booster_17mm_1_Heat();
    Shooter_Speed = uint16_t(Referee.Get_Shoot_Speed() * 10);
    // 发送数据给云台
    CAN2_Chassis_Tx_Gimbal_Data[0] = Referee.Get_ID();
    CAN2_Chassis_Tx_Gimbal_Data[1] = Referee.Get_Game_Stage();
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 2, &Shooter_Barrel_Heat_Limit, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 4, &Shooter_Barrel_Heat, sizeof(uint16_t));
    memcpy(CAN2_Chassis_Tx_Gimbal_Data + 6, &Shooter_Speed, sizeof(uint16_t));
}
#endif

/**
 * @brief can回调函数处理云台发来的数据
 *
 */
Struct_CAN_Referee_Rx_Data_t CAN_Referee_Rx_Data;
#ifdef CHASSIS    
//控制类型字节
uint8_t control_type;

void Class_Chariot::CAN_Chassis_Rx_Gimbal_Callback(uint8_t *Rx_Data)
{   
    Gimbal_Alive_Flag++;
    //底盘坐标系的目标速度
    float gimbal_velocity_y,gimbal_velocity_x;
    float chassis_velocity_x, chassis_velocity_y;
    //目标角速度
    float chassis_omega;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //超电控制类型
    Enum_Supercap_Mode supercap_mode;
    //float映射到int16之后的速度
    uint16_t tmp_velocity_x, tmp_velocity_y;
	uint8_t tmp_omega;

	
    memcpy(&tmp_velocity_x, &Rx_Data[0], sizeof(uint16_t));
    memcpy(&tmp_velocity_y, &Rx_Data[2], sizeof(uint16_t));
    memcpy(&tmp_omega, &Rx_Data[4], sizeof(uint8_t));
    memcpy(&control_type, &Rx_Data[7], sizeof(uint8_t));
            

    gimbal_velocity_x = Math_Int_To_Float(tmp_velocity_x, 0, 0x7FFF, -1 * Chassis.Get_Velocity_X_Max(), Chassis.Get_Velocity_X_Max());
    gimbal_velocity_y = Math_Int_To_Float(tmp_velocity_y, 0, 0x7FFF, -1 * Chassis.Get_Velocity_Y_Max(), Chassis.Get_Velocity_Y_Max());
    chassis_omega = Math_Int_To_Float(tmp_omega,0, 256, -4.0f, 4.0f);      // Chassis_Radius;//映射范围除以五十 云台发的是车体角速度 转为舵轮电机的线速度
    chassis_control_type = (Enum_Chassis_Control_Type)control_type;

//            //获取云台坐标系和底盘坐标系的夹角（弧度制）
//            //角速度前馈，保证小陀螺时走直线
           float Feedback_Angle =  0.0f;
            if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
            {
                Feedback_Angle = -0.025f * Chassis.Get_Spin_Omega();
            }
            else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
            {
                Feedback_Angle = 0.025f * Chassis.Get_Spin_Omega();
            }
            else
            {
                Feedback_Angle = 0.0f;
            }

           float derta_angle;
           Chassis_Angle = Motor_Yaw.Get_Now_Radian();
           derta_angle = -(Reference_Radian - Chassis_Angle) + Offset_Angle + Feedback_Angle;
           derta_angle = derta_angle < 0 ? (derta_angle + 2 * PI) : derta_angle;

           // 云台坐标系的目标速度转为底盘坐标系的目标速度 正常情况下不会更正，只有有偏移未矫正或者小陀螺或反小陀螺会更正
           //无论云台如何动，dt7传的数据可以直接对应底盘车体运动行进，而不是完全以头为正方向
						//从笛卡尔系到右手系
           chassis_velocity_y = -1.0f * ((float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_y * sin(derta_angle)));
           chassis_velocity_x = 1.0f * ((float)(gimbal_velocity_x * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle)));

           if(chassis_omega < 0.5f && chassis_omega > -0.5f)chassis_omega = 0;//限幅


            // chassis_velocity_x = gimbal_velocity_y;
            // chassis_velocity_y = -gimbal_velocity_x;
            
            //设定底盘目标速度
            Chassis.Set_Target_Velocity_X(chassis_velocity_x);
            Chassis.Set_Target_Velocity_Y(chassis_velocity_y);
						
            //设定底盘控制类型
            Chassis.Set_Chassis_Control_Type(chassis_control_type);            
//            Chassis.Set_Supercap_Mode(supercap_mode);
            Chassis.Set_Supercap_Mode(Supercap_ENABLE);

}
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


/**
 * @brief can回调函数给地盘发送数据
 *
 */
#ifdef GIMBAL
//控制类型字节
uint8_t control_type;
void Class_Chariot::CAN_Gimbal_Tx_Chassis_Callback()
{
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0; 
    //映射之后的目标速度 int16_t
    int16_t tmp_chassis_velocity_x = 0, tmp_chassis_velocity_y = 0, tmp_chassis_omega = 0;
    float chassis_omega = 0;
    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;
    //超电控制类型
    uint8_t Supercap_Mode;
    //控制类型字节
    MiniPC_Status = MiniPC.Get_MiniPC_Status();
    chassis_velocity_x = Chassis.Get_Target_Velocity_X();
    chassis_velocity_y = Chassis.Get_Target_Velocity_Y();
    chassis_omega = Chassis.Get_Target_Omega();
    chassis_control_type = Chassis.Get_Chassis_Control_Type();
    Supercap_Mode = MiniPC.Get_Supercap_Mode();
    //设定速度
    tmp_chassis_velocity_x = Math_Float_To_Int(chassis_velocity_x,-4.f , 4.f ,-450,450);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data, &tmp_chassis_velocity_x, sizeof(int16_t));

    tmp_chassis_velocity_y = Math_Float_To_Int(chassis_velocity_y,-4.f , 4.f ,-450,450);
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 2, &tmp_chassis_velocity_y, sizeof(int16_t));
    
    tmp_chassis_omega = -Math_Float_To_Int(chassis_omega,-4.f ,4.f ,-200,200);//随动环 逆时针为正所以加负号
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 4, &tmp_chassis_omega, sizeof(int16_t));

    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 6,&Supercap_Mode ,sizeof(uint8_t));//超电

    control_type =  (uint8_t)chassis_control_type;
    memcpy(CAN3_Gimbal_Tx_Chassis_Data + 7,&control_type ,sizeof(uint8_t));

}
#endif
/**
 * @brief 底盘控制逻辑
 *
 */  		
float Offset_K = 0.175f;
//#ifdef GIMBAL
#ifdef TEST
void Class_Chariot::Control_Chassis()
{
    //遥控器摇杆值
    float dr16_l_x = 0, dr16_l_y = 0;
    float vt13_l_x = 0, vt13_l_y = 0;
    
    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float chassis_omega = 0;  
#ifdef USE_DR16
	
    //排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;

		// 设定矩形到圆形映射进行控制，因为遥控器理想是圆，但实际数据是正方形，如果45°移动，将会是1.414倍，所以要映射
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
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN_NePositive);
            chassis_omega = -Chassis.Get_Spin_Omega();
            if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                chassis_omega = Chassis.Get_Spin_Omega();
            }
        }	
#endif
 
#ifdef USE_VT13
       // 排除遥控器死区
     vt13_l_x = (Math_Abs(VT13.Get_Left_X()) > DR16_Dead_Zone) ? VT13.Get_Left_X() : 0;
     vt13_l_y = (Math_Abs(VT13.Get_Left_Y()) > DR16_Dead_Zone) ? VT13.Get_Left_Y() : 0;

        // 设定矩形到圆形映射进行控制
     chassis_velocity_x = vt13_l_x * sqrt(1.0f - vt13_l_y * vt13_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
     chassis_velocity_y = vt13_l_y * sqrt(1.0f - vt13_l_x * vt13_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();

        // 键盘遥控器操作逻辑
        if (VT13.Get_Switch() == VT13_Switch_Status_Left)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            chassis_omega = -Chassis.Get_Spin_Omega();
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Middle)
        {
            // 底盘随动
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_FLLOW);
        }
        if (VT13.Get_Switch() == VT13_Switch_Status_Right)
        {
            Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_SPIN);
            chassis_omega = Chassis.Get_Spin_Omega();
        }
#endif

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);//前x正，左y正
    Chassis.Set_Target_Omega(chassis_omega);
}
#endif

/**
* @brief 底盘直接接收DR16测试
*
*/
#ifdef TEST
float target[2];
void Class_Chariot::Control_Chassis_Test()
{
    //遥控器摇杆值
    float dr16_l_x = 0, dr16_l_y = 0,dr16_r_x = 0;

    //底盘坐标系速度目标值 float
    float chassis_velocity_x = 0, chassis_velocity_y = 0;
    float gimbal_velocity_x=0,gimbal_velocity_y=0;
    float chassis_omega = 0; 

    //底盘控制类型
    Enum_Chassis_Control_Type chassis_control_type;

    //排除遥控器死区
    dr16_l_x = (Math_Abs(DR16.Get_Left_X()) > DR16_Dead_Zone) ? DR16.Get_Left_X() : 0;
    dr16_l_y = (Math_Abs(DR16.Get_Left_Y()) > DR16_Dead_Zone) ? DR16.Get_Left_Y() : 0;    
    dr16_r_x = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;
	// 设定矩形到圆形映射进行控制，因为遥控器理想是圆，但实际数据是正方形，如果45°移动，将会是1.414倍，所以要映射
	gimbal_velocity_x = dr16_l_x * sqrt(1.0f - dr16_l_y * dr16_l_y / 2.0f) * Chassis.Get_Velocity_X_Max();
    gimbal_velocity_y = dr16_l_y * sqrt(1.0f - dr16_l_x * dr16_l_x / 2.0f) * Chassis.Get_Velocity_Y_Max();	
	//    //统一一下，进行限幅，但我感觉不太需要		因为初始化变量里面有一个值进行限制
//        if(chassis_velocity_x>4.0f)
//        {
//            chassis_velocity_x=4.0f;
//        }
//        else if(chassis_velocity_x<-4.0f)
//        {
//            chassis_velocity_x=-4.0f;
//        } 
//        if(chassis_velocity_y>4.0f)
//        {
//            chassis_velocity_y=4.0f;
//        }
//        else if(chassis_velocity_y<-4.0f)
//        {
//            chassis_velocity_y=-4.0f;
//        }
//        
        // 键盘遥控器操作逻辑
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_MIDDLE) // 左中 随动模式
        {
            // 底盘随动
            chassis_control_type=Chassis_Control_Type_FLLOW;
            chassis_omega = -dr16_r_x * Chassis.Get_Omega_Max();	
        }
        if (DR16.Get_Left_Switch() == DR16_Switch_Status_UP) // 左上 小陀螺模式
        {
            chassis_control_type=Chassis_Control_Type_SPIN_Positive;
            chassis_omega = -Chassis.Get_Spin_Omega();
					
                if (DR16.Get_Right_Switch() == DR16_Switch_Status_DOWN) // 右下 小陀螺反向
            {
                chassis_control_type=Chassis_Control_Type_SPIN_NePositive;
                chassis_omega = Chassis.Get_Spin_Omega();
            }        				
        }


//        if(chassis_omega>4.0f)
//        {
//            chassis_omega=4.0f;
//        }
//        else if(chassis_omega<4.0f)
//        {
//            chassis_omega=-4.0f;
//        }
        #ifdef OLD

        //获取云台坐标系和底盘坐标系的夹角（弧度制）
        //角速度前馈，保证小陀螺时走直线
        float Feedback_Angle =  0.0f;
        if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
        {
            Feedback_Angle = -0.025f * Math_Int_To_Float(chassis_omega,0,0xFF,-1 * 20.0f,20.0f);
        }
        else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
        {
            Feedback_Angle = 0.025f * Math_Int_To_Float(chassis_omega,0,0xFF,-1 * 20.0f,20.0f);
        }
        else
        {
            Feedback_Angle = 0.0f;
        }

    float derta_angle;
    Chassis_Angle = Motor_Yaw.Get_Now_Radian();
    derta_angle = Reference_Angle - Chassis_Angle + Offset_Angle + Feedback_Angle;
    derta_angle = derta_angle < 0 ? (derta_angle + 2 * PI) : derta_angle;

    // 云台坐标系的目标速度转为底盘坐标系的目标速度 正常情况下不会更正，只有有偏移未矫正或者小陀螺或反小陀螺会更正
    //无论云台如何动，dt7传的数据可以直接对应底盘车体运动行进，而不是完全以头为正方向
    gimbal_velocity_x = 1.0f * ((float)(gimbal_velocity_x * cos(derta_angle) - gimbal_velocity_x * sin(derta_angle)));
    gimbal_velocity_y = 1.0f * ((float)(gimbal_velocity_y * sin(derta_angle) + gimbal_velocity_y * cos(derta_angle)));
    if(chassis_omega < 0.5f && chassis_omega > -0.5f)chassis_omega = 0;//限幅
    //小陀螺行进，我感觉一般不会进入这个判断
    #endif
    chassis_velocity_x=gimbal_velocity_y;
    chassis_velocity_y=-gimbal_velocity_x;

    Chassis.Set_Target_Velocity_X(chassis_velocity_x);
    Chassis.Set_Target_Velocity_Y(chassis_velocity_y);//前x正，左y正
    Chassis.Set_Target_Omega(chassis_omega); 
    
    Chassis.Set_Chassis_Control_Type(chassis_control_type);    
}

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
    float dr16_y = 0, dr16_r_y = 0;
    float vt13_y = 0, vt13_r_y = 0;//把所有主要的变量统一放在交互层函数开头，便于后面查找
	
    tmp_gimbal_yaw = Gimbal.Get_Target_Yaw_Angle();//增量环
	tmp_gimbal_pitch = Gimbal.Motor_Pitch.Get_Target_Angle();//在第一步直接获取Target，后面就只是不断优化	
#ifdef USE_DR16
    // 排除遥控器死区
	dr16_y = (Math_Abs(DR16.Get_Right_X()) > DR16_Dead_Zone) ? DR16.Get_Right_X() : 0;//DR16和VT13共用一套死区
    dr16_r_y = (Math_Abs(DR16.Get_Right_Y()) > DR16_Dead_Zone) ? DR16.Get_Right_Y() : 0;
	
	//已经获取了右杆的数据，接下来考虑遥控器上面的按键
    if (DR16.Get_Left_Switch() == DR16_Switch_Status_DOWN) // 左下 上位机
    {
        Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
    }
    else // 其余位置都是遥控器控制， 左中是普通遥控模式，非自瞄模式   
    { 
			Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
			
			
			// 遥控器操作逻辑
			tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution;//共用一套灵敏度系数，调参调这个
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
			if(tmp_gimbal_pitch < -25.0f)tmp_gimbal_pitch = -25.0f;//框定范围

    }
#endif	
	

#ifdef USE_VT13
        // 排除遥控器死区
        vt13_y = (Math_Abs(VT13.Get_Right_X()) > DR16_Dead_Zone) ? VT13.Get_Right_X() : 0;//共用一套死区
        vt13_r_y = (Math_Abs(VT13.Get_Right_Y()) > DR16_Dead_Zone) ? VT13.Get_Right_Y() : 0;
				

        if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_DISABLE)//VT13是按键控制，所以比DR16多了点东西
            Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
        if (VT13.Get_Button_Right() == VT13_Button_TRIG_FREE_PRESSED) //
        {
            if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_NORMAL)//从这里开始看，是一致的
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_MINIPC);
            else if (Gimbal.Get_Gimbal_Control_Type() == Gimbal_Control_Type_MINIPC)
			{
                Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_NORMAL);
								// 遥控器操作逻辑
				tmp_gimbal_yaw -= dr16_y * DR16_Yaw_Angle_Resolution;//共用一套灵敏度系数，调参调这个
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
				if(tmp_gimbal_pitch < -25.0f)tmp_gimbal_pitch = -25.0f;//框定范围							
			}
 
        }
#endif		
    //todo：键鼠   
		// 设定目标角度
      Gimbal.Set_Target_Yaw_Angle(tmp_gimbal_yaw);
      Gimbal.Set_Target_Pitch_Angle(tmp_gimbal_pitch);		
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
#ifdef USE_DR16
        // 左上 开启摩擦轮和发射机构
        if (DR16.Get_Right_Switch() == DR16_Switch_Status_UP)
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
            Fric_Status = Fric_Status_OPEN;

            if (DR16.Get_Yaw() > -0.2f && DR16.Get_Yaw() < 0.2f)
            {
                Shoot_Flag = 0;
            }
            if (DR16.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if (DR16.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            }
        }
        else
        {
            Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
            Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
            Fric_Status = Fric_Status_CLOSE;
        }
#endif

#ifdef USE_VT13
       // 开启摩擦轮和发射机构
        if (VT13.Get_Button_Left() == VT13_Button_TRIG_FREE_PRESSED)
        {
            if (Fric_Status == Fric_Status_OPEN)
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_DISABLE);
                Fric_Status = Fric_Status_CLOSE;
            }
            else
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_CEASEFIRE);
                Booster.Set_Friction_Control_Type(Friction_Control_Type_ENABLE);
                Fric_Status = Fric_Status_OPEN;
            }
        }
        if (Fric_Status == Fric_Status_OPEN)
        {
            if (VT13.Get_Yaw() > -0.2f && VT13.Get_Yaw() < 0.2f)
            {
                Shoot_Flag = 0;
            }
            if (VT13.Get_Yaw() < -0.8f && Shoot_Flag == 0) // 单发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_SINGLE);
                Shoot_Flag = 1;
            }
            if (VT13.Get_Yaw() > 0.8f && Shoot_Flag == 0) // 五连发
            {
                Booster.Set_Booster_Control_Type(Booster_Control_Type_MULTI);
                Shoot_Flag = 1;
            }
        }

#endif
}
#endif

/**
 * @brief 计算回调函数
 *
 */

void Class_Chariot::TIM_Calculate_PeriodElapsedCallback()
{
#ifdef CHASSIS
#ifdef OLD
        // 底盘给云台发消息
        CAN_Chassis_Tx_Gimbal_Callback();

		if (Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_FLLOW)
		{
				// 随动环
				Chassis_Angle = Motor_Yaw.Get_Now_Angle();

				PID_Chassis_Follow.Set_Target(Reference_Angle);
				PID_Chassis_Follow.Set_Now(Chassis_Angle);
				if(Reference_Angle - Chassis_Angle>180.0f)
				{
					PID_Chassis_Follow.Set_Target(Reference_Angle-360.0f);
				}
				else if(Reference_Angle - Chassis_Angle <-180.0f)
				{
					PID_Chassis_Follow.Set_Target(Reference_Angle +180.0f);
				}
				else
				{
					
				}
				PID_Chassis_Follow.TIM_Adjust_PeriodElapsedCallback();

//				Chassis.Filter_Omega.Set_Now(-PID_Chassis_Follow.Get_Out());//如有需要进行一次傅里叶滤波
//			  Chassis.Filter_Omega.TIM_Adjust_PeriodElapsedCallback();

				Chassis.Set_Target_Omega(PID_Chassis_Follow.Get_Out());
		}
		
		else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_Positive)
		{
			Chassis.Set_Target_Omega(-Chassis.Get_Spin_Omega());
		}
		
		else if(Chassis.Get_Chassis_Control_Type() == Chassis_Control_Type_SPIN_NePositive)
		{
			Chassis.Set_Target_Omega(Chassis.Get_Spin_Omega());
		}
		else
		{
			Chassis.Set_Target_Omega(0.0f);
		}

		// Chassis.Set_Sprint_Status(Sprint_Status);		
        //云台，随动掉线保护
        #ifdef BATTLE
        if(Get_Gimbal_Status() == DR16_Status_ENABLE || Referee.Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE)		
        #endif

		// if(Get_Gimbal_Status() == DR16_Status_ENABLE)
        if(Get_Gimbal_Status()==Gimbal_Status_ENABLE)
        //if(DR16.Get_DR16_Status()==DR16_Status_ENABLE)
        // if(Get_Chassis_Status() == Chassis_Status_ENABLE)
        {
            Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);//还有飞坡前馈没写
        }
        else
        {
            for(int i = 0; i < 4; i++)
				{
                Chassis.Motor_Wheel[i].Set_Out(0.0f);
                Chassis.Motor_Steer[i].Set_Out(0.0f);
				}
        }
        //DWT_SysTimeUpdate();
        #endif
        Chassis.TIM_Calculate_PeriodElapsedCallback(Sprint_Status);//还有飞坡前馈没写
        Chassis.Supercap.Set_Working_Status(Working_Status_OFF);
    #elif defined(GIMBAL)

        //各个模块的分别解算
        Gimbal.TIM_Calculate_PeriodElapsedCallback();
        Booster.TIM_Calculate_PeriodElapsedCallback();
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
/**
 * @brief 控制回调函数
 *
 */
#ifdef TEST
void Class_Chariot::TIM_Control_Callback()
{
    //底盘，云台，发射机构控制逻辑
    Control_Chassis();
    //Control_Gimbal();
    //Control_Booster();
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
	
	Motor_Yaw.TIM_Alive_PeriodElapsedCallback();//为解决掉帧问题，必须高频率检测	
	
    if (mod50 == 50)
    {
        mod50_mod3++;
        //TIM_Unline_Protect_PeriodElapsedCallback();
        #ifdef CHASSIS
            
            Chassis.Supercap.TIM_Alive_PeriodElapsedCallback();
            for (auto& wheel : Chassis.Motor_Wheel) {
                wheel.TIM_Alive_PeriodElapsedCallback();
            }
            for (auto& steer : Chassis.Motor_Steer) {
                steer.TIM_Alive_PeriodElapsedCallback();
                steer.TIM_Alive_PeriodElapsedCallback_MA600();
            }          
            if(mod50_mod3%3 == 0)
            {
							Referee.TIM1msMod50_Alive_PeriodElapsedCallback();
                TIM1msMod50_Gimbal_Communicate_Alive_PeriodElapsedCallback();
                mod50_mod3 = 0;
            }
            // if(Get_Gimbal_Status() == Gimbal_Status_DISABLE){
            //     Chassis.Set_Target_Velocity_X(0);
            //     Chassis.Set_Target_Velocity_Y(0);
            //     Chassis.Set_Target_Omega(0);
            // }   
					    #ifdef TEST
            DR16.TIM1msMod50_Alive_PeriodElapsedCallback();							
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
            Gimbal.Boardc_BMI.TIM1msMod50_Alive_PeriodElapsedCallback();

            Booster.Motor_Driver.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Left.TIM_Alive_PeriodElapsedCallback();
            Booster.Motor_Friction_Right.TIM_Alive_PeriodElapsedCallback();
						
			MiniPC.TIM1msMod50_Alive_PeriodElapsedCallback();

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
    if(Get_Gimbal_Status() == Gimbal_Status_DISABLE)
    {
        Chassis.Set_Target_Velocity_X(0);
        Chassis.Set_Target_Velocity_Y(0);
        Chassis.Set_Target_Omega(0);
    }
        
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
//#ifdef GIMBAL
#ifdef TEST
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
//            Chariot->Booster.Set_Booster_Control_Type(Booster_Control_Type_DISABLE);
//            Chariot->Gimbal.Set_Gimbal_Control_Type(Gimbal_Control_Type_DISABLE);
            Chariot->Chassis.Set_Chassis_Control_Type(Chassis_Control_Type_DISABLE);

            if(Chariot->DR16.Get_DR16_Status() == DR16_Status_ENABLE)
            {
                Chariot->Chassis.Set_Chassis_Control_Type(Chariot->Get_Pre_Chassis_Control_Type());
//              Chariot->Gimbal.Set_Gimbal_Control_Type(Chariot->Get_Pre_Gimbal_Control_Type());
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
//            Chariot->Set_Pre_Chassis_Control_Type(Chariot->Chassis.Get_Chassis_Control_Type());
//            Chariot->Set_Pre_Gimbal_Control_Type(Chariot->Gimbal.Get_Gimbal_Control_Type());

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
