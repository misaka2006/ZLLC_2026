#include "crt_steering_wheel.h"
#include "config.h""
Class_Steering_Wheel steering_wheel;
void Class_Steering_Wheel::CAN_RxAgvBoardCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{
    int16_t motion_motor_troque = 0;
    int16_t direct_motor_troque = 0;
    switch (CAN_RxMessage->Header.StdId)
    {
    case 0x02A:
    {
        memcpy(&Power_Management.Motor_Data[0].feedback_omega, CAN_RxMessage->Data, 2);
        memcpy(&direct_motor_troque, CAN_RxMessage->Data + 2, 2);
        Power_Management.Motor_Data[0].torque = direct_motor_troque/1000.0f;
        memcpy(&Power_Management.Motor_Data[1].feedback_omega, CAN_RxMessage->Data + 4, 2);
        memcpy(&motion_motor_troque, CAN_RxMessage->Data + 6, 2);
        Power_Management.Motor_Data[1].torque = motion_motor_troque/1000.f;
    }
    break;
    case 0x02B:
    {
        memcpy(&Power_Management.Motor_Data[2].feedback_omega, CAN_RxMessage->Data, 2);
        memcpy(&direct_motor_troque, CAN_RxMessage->Data + 2, 2);
        Power_Management.Motor_Data[2].torque = direct_motor_troque/1000.0f;
        memcpy(&Power_Management.Motor_Data[3].feedback_omega, CAN_RxMessage->Data + 4, 2);
        memcpy(&motion_motor_troque, CAN_RxMessage->Data + 6, 2);
        Power_Management.Motor_Data[3].torque = motion_motor_troque/1000.f;
    }
    break;
    case 0x02C:
    {
        memcpy(&Power_Management.Motor_Data[4].feedback_omega, CAN_RxMessage->Data, 2);
        memcpy(&direct_motor_troque, CAN_RxMessage->Data + 2, 2);
        Power_Management.Motor_Data[4].torque = direct_motor_troque/1000.0f;
        memcpy(&Power_Management.Motor_Data[5].feedback_omega, CAN_RxMessage->Data + 4, 2);
        memcpy(&motion_motor_troque, CAN_RxMessage->Data + 6, 2);
        Power_Management.Motor_Data[5].torque = motion_motor_troque/1000.f;
    }
    break;
    case 0x02D:
    {
        memcpy(&Power_Management.Motor_Data[6].feedback_omega, CAN_RxMessage->Data, 2);
        memcpy(&direct_motor_troque, CAN_RxMessage->Data + 2, 2);
        Power_Management.Motor_Data[6].torque = direct_motor_troque/1000.0f;
        memcpy(&Power_Management.Motor_Data[7].feedback_omega, CAN_RxMessage->Data + 4, 2);
        memcpy(&motion_motor_troque, CAN_RxMessage->Data + 6, 2);
        Power_Management.Motor_Data[7].torque = motion_motor_troque/1000.f;
    }
    break;
    case 0x03A:
    {
        memcpy(&Power_Management.Motor_Data[0].torque, CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[1].torque, CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x03B:
    {
        memcpy(&Power_Management.Motor_Data[2].torque, CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[3].torque, CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x03C:
    {
        memcpy(&Power_Management.Motor_Data[4].torque, CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[5].torque, CAN_RxMessage->Data + 4, 4);
    }
    break;
    case 0x03D:
    {
        memcpy(&Power_Management.Motor_Data[6].torque, CAN_RxMessage->Data, 4);
        memcpy(&Power_Management.Motor_Data[7].torque, CAN_RxMessage->Data + 4, 4);
    }
    break;
    #if defined(AGV_BOARD_A) || defined(AGV_BOARD_C) || defined(AGV_BOARD_D)
    case 0x03F:
    {
        memcpy(&Power_Management.Motor_Data[0].output,CAN_RxMessage->Data,2);
        memcpy(&Power_Management.Motor_Data[2].output,CAN_RxMessage->Data+2,2);
        memcpy(&Power_Management.Motor_Data[4].output,CAN_RxMessage->Data+4,2);
        memcpy(&Power_Management.Motor_Data[6].output,CAN_RxMessage->Data+6,2);
    }
    break;
    case 0x03E:
    {
        memcpy(&Power_Management.Motor_Data[1].output,CAN_RxMessage->Data,2);
        memcpy(&Power_Management.Motor_Data[3].output,CAN_RxMessage->Data+2,2);
        memcpy(&Power_Management.Motor_Data[5].output,CAN_RxMessage->Data+4,2);
        memcpy(&Power_Management.Motor_Data[7].output,CAN_RxMessage->Data+6,2);
    }
    break;
    #endif
    }
    
}

// 这里要根据帧ID判断是功率数据还是角度速度数据
//float velocity_x, velocity_y, velocity, theta;
//float last_angle=0;
float power;
float power_max = 70;
void Class_Steering_Wheel::CAN_RxChassisCallback(Struct_CAN_Rx_Buffer *CAN_RxMessage)
{   //角度是以编码值为单位的（单圈编码值为8191），速度是以RPM为单位的
    uint16_t tmp_angle_encoder = 0;
    int16_t tmp_velocity_rpm = 0;
    Enum_Steering_Cmd_Status Cmd_Status;

    CanRX_Chassis_Cmd_Flag++;
    #ifdef OLD
    if (CAN_RxMessage->Header.StdId == AGV_BOARD_ID)
    {
	    memcpy(&tmp_angle_encoder,CAN_RxMessage->Data,2);
        memcpy(&tmp_velocity_rpm,CAN_RxMessage->Data+2,2);
        //memcpy(&Cmd_Status,CAN_RxMessage->Data+4,1);
        this->Target_Angle = Math_Int_To_Float(tmp_angle_encoder,0,0xffff,0.0f,8191.0f) * 360.0f / 8191.0f;
        this->Target_Omega = (float)tmp_velocity_rpm * 6.0f;
        this->Target_Velocity = this->Target_Omega / 180.0f * PI * Wheel_Diameter / 2.0f;//rpm转为m/s
        // this->Steering_Cmd_Status = Cmd_Status;
				
    }
    #endif
    #if defined(AGV_BOARD_A) || defined(AGV_BOARD_D)
    uint8_t board_id = AGV_BOARD_ID;
    if(CAN_RxMessage->Header.StdId == 0x1AU)
    {
        if(board_id == 0x1DU)
        {
            memcpy(&tmp_angle_encoder,CAN_RxMessage->Data+4,2);
            memcpy(&tmp_velocity_rpm,CAN_RxMessage->Data+6,2);
        }
        else if(board_id == 0x1AU)
        {
            memcpy(&tmp_angle_encoder,CAN_RxMessage->Data,2);
            memcpy(&tmp_velocity_rpm,CAN_RxMessage->Data+2,2);
        }

        this->Target_Angle = Math_Int_To_Float(tmp_angle_encoder,0,0xffff,0.0f,8191.0f) * 360.0f / 8191.0f;
        this->Target_Omega = (float)tmp_velocity_rpm * 6.0f;
        this->Target_Velocity = this->Target_Omega / 180.0f * PI * Wheel_Diameter / 2.0f;//rpm转为m/s
    }
    #endif
    #if defined(AGV_BOARD_B) || defined(AGV_BOARD_C)
    if(CAN_RxMessage->Header.StdId == 0x1BU)
    {
			uint8_t board_id = AGV_BOARD_ID;
        if(board_id == 0x1CU)
        {
            memcpy(&tmp_angle_encoder,CAN_RxMessage->Data+4,2);
            memcpy(&tmp_velocity_rpm,CAN_RxMessage->Data+6,2);
        }
        else if(board_id == 0x1BU)
        {
            memcpy(&tmp_angle_encoder,CAN_RxMessage->Data,2);
            memcpy(&tmp_velocity_rpm,CAN_RxMessage->Data+2,2);
        }

        this->Target_Angle = Math_Int_To_Float(tmp_angle_encoder,0,0xffff,0.0f,8191.0f) * 360.0f / 8191.0f;
        this->Target_Omega = (float)tmp_velocity_rpm * 6.0f;
        this->Target_Velocity = this->Target_Omega / 180.0f * PI * Wheel_Diameter / 2.0f;//rpm转为m/s
    }
    #endif

    if (CAN_RxMessage->Header.StdId == 0x01E)
    {
        uint8_t power_max = 0;
        memcpy(&power_max, CAN_RxMessage->Data, 1);
        Power_Management.Max_Power = (float)power_max;
	    memcpy(&Power_Management.Actual_Power,CAN_RxMessage->Data+1,4);
        memcpy(&Cmd_Status,CAN_RxMessage->Data+5,1);
        this->Steering_Cmd_Status = Cmd_Status;
        uint8_t status = 0;
        memcpy(&status,CAN_RxMessage->Data+6,1);
        Power_Limit.Set_Control_Status(status);
    }
}

void Class_Steering_Wheel::CanRX_Chassis_TIM_Alive_PeriodElapsedCallback()
{
    //判断是否有新的指令
    if(CanRX_Chassis_Cmd_Flag == CanRX_Chassis_Cmd_Pre_Flag)
    {
        //与底盘通信断开
        CanRX_Chassis_Cmd_Status = CanRX_Chassis_Cmd_Status_DISABLE;
    }
    else
    {
        //与底盘通信连接
        CanRX_Chassis_Cmd_Status = CanRX_Chassis_Cmd_Status_ENABLE;
    }
    CanRX_Chassis_Cmd_Pre_Flag = CanRX_Chassis_Cmd_Flag;
}

void Class_Steering_Wheel::CanRX_AGV_Board_TIM_Alive_PeriodElapsedCallback()
{
    //判断是否有新的指令
    if(CanRX_AGV_Board_Flag == CanRX_AGV_Board_Pre_Flag)
    {
        //与其他舵通信断开
        CanRX_AGV_Board_Flag_Status = CanRX_AGV_Board_Status_DISABLE;
    }
    else
    {
        //与其他舵通信连接
        CanRX_AGV_Board_Flag_Status = CanRX_AGV_Board_Status_ENABLE;
    }
    CanRX_AGV_Board_Pre_Flag = CanRX_AGV_Board_Flag;
}
void Class_Steering_Wheel::Init()
{
    memset(&Power_Management, 0, sizeof(Power_Management));

    // todo:待调参

    Motion_Motor.PID_Omega.Init(7.5, 0, 0, 0, 0, 16384);
    Motion_Motor.Init(&hcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, MOT_OUTPUT_TO_ROTOR_RATIO);

    Directive_Motor.PID_Angle.Init(20, 0, 0, 0, 0, 16384);
    Directive_Motor.PID_Omega.Init(25, 0, 0, 0, 0, 16384);

#ifdef DEBUG_DIR_SPEED
    Directive_Motor.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA, 8);
#else
    Directive_Motor.Init(&hcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_ANGLE, DIR_OUTPUT_TO_ROTOR_RATIO);
#endif 

    Encoder.Init(&hcan1, static_cast<Enum_Encoder_ID>(ENCODER_ID));
    Power_Limit.Init();
}
//红色：R 
//黑色：T
//黄色：GND
