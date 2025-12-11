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
float* get_now_motor_angles(Class_Gimbal* Gimbal);
void move_result(float q[6], uint8_t axis, float s, float pos[3]);
float get_s_at(uint32_t t_ms);
void calculate_trajectory_xyz(float q_start[6], uint8_t axis, float trajectory_xyz[600][3]);
uint32_t ikine_trajectory(float trajectory_xyz[600][3], float rpy_target[3], float q_solution[600][6], float q_start[6]);
float test_get_s_at();
#endif // ARM_MODEL_H