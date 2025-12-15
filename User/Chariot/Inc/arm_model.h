#ifndef ARM_MODEL_H
#define ARM_MODEL_H

#include "robotics.h"
#include "matrix.h"
#include "user_lib.h"

/* Exported variables --------------------------------------------------------*/
extern robotics::Serial_Link<6> robot;
extern robotics::Link links[6];
class Class_Gimbal;
/* Exported function declarations --------------------------------------------*/
robotics::Serial_Link<6> CreateMyRobot();

bool SolveRobotIK_Iterative_Iterative(float target_pos[3], float target_rpy[3], float q_result[6], float now_angle[6]);

void model_to_control(float model_angles[6], float control_angles[6]);
void motor_to_model(float control_angles[6], float model_angles[6], float cali_offset);

float multi_to_single(float radian);
void show_FK_result(float joint_angles[6], float xyz_rpy[6]);

static float normalize_angle(float angle);
uint8_t ikine_pieper_solutions(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8]);
uint8_t solution_filter(Matrixf<6, 1> solutions[8], bool valid[8]);
uint8_t get_best_solution_index(Matrixf<6, 1> solutions[8], bool valid[8], float current_angle[6]);
float *get_now_motor_angles(Class_Gimbal *Gimbal);
void move_result(float q[6], uint8_t axis, float s, float pos[3]);
float get_s_at(uint32_t t_ms);
void calculate_trajectory_xyz(float q_start[6], uint8_t axis, float trajectory_xyz[600][3]);
uint32_t ikine_trajectory(float trajectory_xyz[600][3], float rpy_target[3], float q_solution[600][6], float q_start[6]);
float test_get_s_at();

/**
 * @brief DH建模类，主要用于模型的正逆解算，内置几个辅助函数
 *
 */
class Class_DH_model
{
    robotics::Serial_Link<6> DH_arm_model;
    float qmin[6];
    float qmax[6];

public:
    Class_DH_model(robotics::Link _links[6], float _qmin[6], float _qmax[6]) : DH_arm_model(_links)
    {
        for (int i = 0; i < 6; i++)
        {
            qmin[i] = _qmin[i];
            qmax[i] = _qmax[i];
        }
    };

    bool Fkine(float joint_angles[6], float pos[3], float rpy[3]);
    bool Fkine(float joint_angles[6], float T[4][4]);
    uint8_t Ikine_Pieper(float pos_target[3], float rpy_target[3], float q_solution[8][6], bool valid[8]);
    float Ikine_Pieper_Best(float pos_target[3], float rpy_target[3], float q_start[6], float q_solution[6]);

private:
    uint8_t Ikine_Pieper_calculator(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8]);

    bool check_valid(Matrixf<6, 1> solution);
    bool check_valid(float q[6]);
    uint8_t valid_count(Matrixf<6, 1> solution[8], bool valid[8]);

    float Euclidean_Distance(float q1[6], float q2[6]);
    float Euclidean_Distance(Matrixf<6, 1> q1, Matrixf<6, 1> q2);
};

class Class_Trajectory_Tracer
{
public:
    Class_Trajectory_Tracer(Class_DH_model _dh_model, Class_Gimbal* _Gimbal) : dh_model(_dh_model)
    {
        Gimbal = _Gimbal;
    };

    
    float get_s_at(uint32_t t_ms);
    float test_get_s_at();
    void get_pos_at(float q_start[6], uint8_t axis, float s, float pos[3]);

    uint32_t Trajectory_Generator(float q_start[6], uint8_t axis, float trajectory_xyz[600][3]);
    uint32_t Trajectory_Ikine(float q_start[6], float rpy_target[3], float trajectory_xyz[600][3], float q_solution[600][6]);
private:
    Class_DH_model dh_model;
    Class_Gimbal* Gimbal;

    float cali_offset = Gimbal->Get_Roll_Cali_Offset() + 1.514f;  // roll_1轴校准误差

    float now_motor_angles[6];
    float now_model_angles[6];

    float trajectory_xyz_buffer[600][3];
    float q_solution_buffer[600][6];


    void model_to_control(float model_angles[6], float control_angles[6]);
    void motor_to_model(float motor_angles[6], float model_angles[6]);
    
    float normalize_angle(float angle);

};
#endif // ARM_MODEL_H