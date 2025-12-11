/**
 * @file arm_model.cpp
 * @author hsl
 * @brief 机械臂机器人学模型，用于正逆运动学计算以及轨迹规划
 * @version 0.1
 * @date 2025-12-03
 *
 * @copyright ZLLC 2026
 *
 */
#include "robotics.h"
#include "matrix.h"
#include "utils.h"
#include "arm_model.h"
#include "crt_gimbal.h"
#ifndef PI
#define PI 3.1415926535f
#endif

using namespace robotics;
using namespace matrixf;

/*----------------------variables-----------------------*/
Link links[6];
float qmin[6] = {-2.883f, -0.926f, 0.15f, -3.028f, -1.336f, -PI};
float qmax[6] = {2.883f, 0.926f, 2.133f, 3.028f, 1.336f, PI};
float now_motor_angles[6];
Serial_Link<6> robot = CreateMyRobot();

// =========================================================
// 1. 定义机械臂模型 (基于 DH 参数)
// =========================================================
// Link(theta, d, a, alpha, type, offset, qmin, qmax, ...)
// 构型: Yaw - Pitch - Pitch - Roll - Pitch - Roll
// 数据: d1=8, a2=35, a3=15.1, a4=12.7, a5=11.5, d6=5.0

Serial_Link<6> CreateMyRobot()
{
    links[0] = Link(0, 8.0f, 0, PI / 2, R, 0, -2.883f, 2.883f);

    links[1] = Link(0, 0, 35.0f, 0, R, PI / 2, -0.926f + PI / 2, 0.926f + PI / 2);

    links[2] = Link(0, 0, 0, -PI / 2, R, -PI / 2, 0.15f - PI / 2, 2.133f - PI / 2);

    links[3] = Link(0, 27.8f, 0, PI / 2, R, 0, -3.028f, 3.028f);

    links[4] = Link(0, 0, 0, -PI / 2, R, 0, -1.336f, 1.336f);

    links[5] = Link(0, 24.5f, 0, 0, R, 0, 0, 0);

    // 创建串联机械臂对象
    return Serial_Link<6>(links);
}

// 迭代法求逆运动学，纯fw，狗都不用
bool SolveRobotIK_Iterative(float target_pos[3], float target_rpy[3], float q_result[6], float now_angle[6])
{
    // 构建目标齐次变换矩阵 Td
    // 使用库函数 rpy2t 将欧拉角转为旋转矩阵
    Matrixf<3, 1> rpy;
    rpy[0][0] = target_rpy[0];
    rpy[1][0] = target_rpy[1];
    rpy[2][0] = target_rpy[2];

    Matrixf<4, 4> T_rot = rpy2t(rpy);

    // 设置位置 p2t
    Matrixf<3, 1> pos;
    pos[0][0] = target_pos[0];
    pos[1][0] = target_pos[1];
    pos[2][0] = target_pos[2];

    // 组合 Td = [R P; 0 1]
    // 库中 rp2t 可以直接从 R 和 p 构建 T
    Matrixf<4, 4> Td = rp2t(t2r(T_rot), pos);

    // 2. 设置初始猜测 (Initial Guess)
    // 使用上一帧的角度或当前位置作为初始猜测
    float data[6] = {0};
    for (int i = 0; i < 6; i++)
    {
        if (now_angle[i] != 0)
        {
            data[i] = now_angle[i]; // 使用当前角度作为初始猜测
        }
        else
        {
            data[i] = 0; // 如果没有当前角度，则使用零位
        }
    }
    // 静态变量初始化一次，初始化时使用当前角度作为初始猜测
    static Matrixf<6, 1> q_guess = Matrixf<6, 1>(data);

    // 数值逆运动学 ikine
    // 参数: 目标T, 初始q, 容差tol(mm/rad), 最大迭代次数
    // ikine 内部实现了奇异性处理
    Matrixf<6, 1> q_sol = robot.ikine(Td, q_guess, 1e-2f, 40);

    // 检查是否收敛：计算 FK 看误差
    Matrixf<4, 4> T_check = robot.fkine(q_sol);
    Matrixf<3, 1> p_err = t2p(Td) - t2p(T_check);

    if (p_err.norm() > 1.0f)
    { // 误差大于 1mm 认为失败
        return false;
    }

    // 5. 输出结果并更新猜测
    q_guess = q_sol; // 更新猜测为当前解，用于下一帧热启动
    for (int i = 0; i < 6; i++)
    {
        q_result[i] = q_sol[i][0];
    }

    return true;
}

void model_to_control(float model_angles[6], float control_angles[6]) // 将建模解算出的角度转成电机实际控制的角度，这里的control_angle是用来给云台类中各个关节的目标角度值赋值用的，测试角度映射时可以顺带调用
{
    // 由于电机零点设置与建模时不同，此函数用于将模型中逆运动学得到的关节角度转换为电机控制角度
    control_angles[0] = model_angles[0] * 2.0f;              // J0-Yaw 减速比为2，零点和运动方向与控制层匹配
    control_angles[1] = model_angles[1] + 0.9268f;           // J1-Pitch1 电机直连，
    control_angles[2] = model_angles[2] * -1.5f + 3.43f;     // J2-Pitch2 方向相反，减速比1.5，这里把2.1345f看作零点，求当前角度在此基础上的增量，再取相反数乘减速比
    control_angles[3] = (model_angles[3] - 3.028f) * -50.0f; // J3-Roll 减速比为50，零点偏移173.5°，这里算出的是相对Roll_Min_Radian的增量，调用Gimbal中的Set_Target_Roll_Radian时会自动加上Roll_Min_Radian
    control_angles[4] = model_angles[4] - 1.277f;            // J4-Pitch3 直连
    // roll2的零点关系，暂时写为相等
    control_angles[5] = model_angles[5];
}

void motor_to_model(float motor_angles[6], float model_angles[6], float cali_offset) // 正运动学求解和轨迹规划时会用到，这里的motor_angles使用电机实际反馈的角度
{
    // cali_offset -> J3-Roll的校准偏移量，恒为负数
    float roll_offset = 1.514f + cali_offset; // Roll上电时的位置相对于垂直向下的角度偏移
    // 由于电机零点设置与建模时不同，此函数用于将电机反馈的实际关节角度转换为模型中的关节角度进行正运动学求解
    model_angles[0] = (motor_angles[0] - PI);                       // J0-Yaw
    model_angles[1] = (motor_angles[1] - PI) * 4.0f - 0.9268f;      // J1-Pitch1
    model_angles[2] = -(motor_angles[2] - PI) / 1.5f + 2.1345f;     // J2-Pitch2 方向相反，减速比1.5
    model_angles[3] = -2.0f * (motor_angles[3] - PI - roll_offset); // J3-Roll 减速比为50，零点偏移PI（这个50的减速比真的需要除吗，存疑）
    model_angles[4] = motor_angles[4] - 0.5f * PI - 0.225f;         // J4-Pitch3
    // roll2的零点关系，暂时写为相等
    model_angles[5] = motor_angles[5];
}

void motor_to_model(float motor_angles[6], float model_angles[6], Class_Gimbal *Gimbal)
// 重载，使用云台类中各个关节的Target_Angle作为输入，不需要校准偏移量，未测试
{
    model_angles[0] = Gimbal->Get_Target_Yaw_Radian();                                              // J0-Yaw
    model_angles[1] = Gimbal->Get_Target_Pitch_Radian();                                            // J1-Pitch1
    model_angles[2] = -Gimbal->Get_Target_Pitch_2_Radian();                                         // J2-Pitch2
    model_angles[3] = -(Gimbal->Get_Target_Roll_Radian() - Gimbal->Get_Roll_Min_Radian()) / 100.0f; // J3-Roll
    model_angles[4] = Gimbal->Get_Target_Pitch_3_Radian();                                          // J4-Pitch3
    model_angles[5] = Gimbal->Get_Target_Roll_2_Radian_Single();
}

float *get_now_motor_angles(Class_Gimbal *Gimbal)
// 返回当前的电机角度，调用Gimbal中Motor对象的Get函数
{
    now_motor_angles[0] = Gimbal->Motor_DM_J0_Yaw.Get_Now_Angle();
    now_motor_angles[1] = Gimbal->Motor_DM_J1_Pitch.Get_Now_Angle();
    now_motor_angles[2] = Gimbal->Motor_DM_J2_Pitch_2.Get_Now_Angle();
    now_motor_angles[3] = Gimbal->Motor_DM_J3_Roll.Get_Now_Angle();
    now_motor_angles[4] = Gimbal->Motor_DM_J4_Pitch_3.Get_Now_Angle();
    now_motor_angles[5] = multi_to_single(Gimbal->Motor_6020_J5_Roll_2.Get_Now_Radian());
    return now_motor_angles;
}

/*测试用函数*/
// 测试角度映射辅助函数，将roll_2的多圈转成单圈(0~2PI)
float multi_to_single(float radian)
{
    float single_radian = fmod(radian, 2.0f * PI);
    if (single_radian < 0)
    {
        single_radian += 2.0f * PI;
    }

    return single_radian;
}

void show_FK_result(float joint_angles[6], float xyz_rpy[6])
{
    Matrixf<6, 1> q;
    for (int i = 0; i < 6; i++)
    {
        q[i][0] = joint_angles[i];
    }
    Matrixf<4, 4> T;
    T = robot.fkine(q);
    xyz_rpy[0] = t2p(T)[0][0];   // X
    xyz_rpy[1] = t2p(T)[1][0];   // Y
    xyz_rpy[2] = t2p(T)[2][0];   // Z
    xyz_rpy[3] = t2rpy(T)[0][0]; // Yaw
    xyz_rpy[4] = t2rpy(T)[1][0]; // Pitch
    xyz_rpy[5] = t2rpy(T)[2][0]; // Roll
}

static float normalize_angle(float angle)
{
    // 使用 user_lib.h 中的宏
    return rad_format(angle);
}

/* *解析法求解器，人人都爱用，实测算出一组解需耗时3ms，计算放前台循环，否则会影响电机通信
 * 一次100mm的轨迹规划大概要算100个点的逆解，总耗时约300ms，速度比较客观，依然放在前台跑
 * pos_target: 目标位置 [x, y, z] 单位 mm
 * rpy_target: 目标姿态 [yaw, pitch, roll] 单位 rad
 * solutions: 输出逆解结果，最多8组解，每组6个关节角
 * return: 实际求解出的解的个数
 */
uint8_t ikine_pieper_solutions(float pos_target[3], float rpy_target[3], Matrixf<6, 1> solutions[8])
{
    Matrixf<3, 1> rpy;
    rpy[0][0] = rpy_target[0];
    rpy[1][0] = rpy_target[1];
    rpy[2][0] = rpy_target[2];

    Matrixf<4, 4> T_rot = rpy2t(rpy);

    // 设置位置 p2t
    Matrixf<3, 1> pos;
    pos[0][0] = pos_target[0];
    pos[1][0] = pos_target[1];
    pos[2][0] = pos_target[2];

    // 组合 Td = [R P; 0 1]
    // 库中 rp2t 可以直接从 R 和 p 构建 T
    Matrixf<4, 4> T_target = rp2t(t2r(T_rot), pos);

    int sol_count = 0;

    // 0. 提取参数
    float d1 = 8.0f;
    float a2 = 35.0f;
    float d4 = 27.8f;
    float d6 = 24.5f;
    float offset2 = PI / 2.0f;

    float Px = T_target[0][3];
    float Py = T_target[1][3];
    float Pz = T_target[2][3];

    // Index [2], [6], [10] 对应矩阵的第3列 (0,2), (1,2), (2,2)
    float Ax = T_target[0][2];
    float Ay = T_target[1][2];
    float Az = T_target[2][2];

    // 计算 Pc
    float Pcx = Px - d6 * Ax;
    float Pcy = Py - d6 * Ay;
    float Pcz = Pz - d6 * Az;

    // 第一阶段：位置求解 q1, q2, q3

    // q1 的两个候选解 (正向/背向)
    float q1_candidates[2];
    q1_candidates[0] = atan2f(Pcy, Pcx);
    q1_candidates[1] = atan2f(-Pcy, -Pcx);

    for (int i = 0; i < 2; i++)
    {
        float q1 = normalize_angle(q1_candidates[i]);

        // 将手腕中心投影到臂平面
        float r = sqrtf(Pcx * Pcx + Pcy * Pcy);
        if (i == 1)
            r = -r; // 背向解，r 取负

        float s = Pcz - d1;

        // --- 几何参数计算 ---
        float D2 = r * r + s * s;
        float D = sqrtf(D2);

        // 余弦定理求内角 phi (a2 与 d4 的夹角)
        float cos_phi = (a2 * a2 + d4 * d4 - D2) / (2.0f * a2 * d4);

        // 检查是否超出工作空间
        if (fabsf(cos_phi) > 1.0001f)
        {
            continue;
        }
        // 数值误差修正
        if (cos_phi > 1.0f)
            cos_phi = 1.0f;
        if (cos_phi < -1.0f)
            cos_phi = -1.0f;

        float phi = acosf(cos_phi);

        // 余弦定理求仰角偏差 beta (a2 与 D 的夹角)
        float cos_beta = (a2 * a2 + D2 - d4 * d4) / (2.0f * a2 * D);

        if (cos_beta > 1.0f)
            cos_beta = 1.0f;
        if (cos_beta < -1.0f)
            cos_beta = -1.0f;

        float beta = acosf(cos_beta);
        float psi = atan2f(s, r);

        // 构造两组 q2/q3 解 (Elbow Up / Elbow Down)
        float q3_opts[2];
        float q2_geom_opts[2];

        // 解A: 肘部向上
        q3_opts[0] = PI - phi;
        q2_geom_opts[0] = psi - beta;

        // 解B: 肘部向下
        q3_opts[1] = phi - PI;
        q2_geom_opts[1] = psi + beta;

        for (int k = 0; k < 2; k++)
        {
            // 计算实际 q2, q3
            float q3 = normalize_angle(q3_opts[k]);
            float q2 = normalize_angle(q2_geom_opts[k] - offset2);

            // 姿态求解 q4, q5, q6

            // 计算前三轴产生的旋转矩阵 R03
            // R03 = R01 * R12 * R23

            // 预计算三角函数
            float c1 = cosf(q1), s1 = sinf(q1);
            float th2 = q2 + offset2; // R12 用的 theta
            float c2 = cosf(th2), s2 = sinf(th2);
            float th3 = q3; // R23 用的 theta
            float offset3 = -PI / 2.0f;
            float th3_mat = q3 + offset3;
            float c3 = cosf(th3_mat), s3 = sinf(th3_mat);

            // 手动构建矩阵 (比通用矩阵乘法快)
            // T01 (alpha=90): [c1 0 s1; s1 0 -c1; 0 1 0]
            Matrixf<3, 3> R01;
            R01[0][0] = c1;
            R01[0][1] = 0;
            R01[0][2] = s1;
            R01[1][0] = s1;
            R01[1][1] = 0;
            R01[1][2] = -c1;
            R01[2][0] = 0;
            R01[2][1] = 1;
            R01[2][2] = 0;

            // T12 (alpha=0): [c2 -s2 0; s2 c2 0; 0 0 1]
            Matrixf<3, 3> R12;
            R12[0][0] = c2;
            R12[0][1] = -s2;
            R12[0][2] = 0;
            R12[1][0] = s2;
            R12[1][1] = c2;
            R12[1][2] = 0;
            R12[2][0] = 0;
            R12[2][1] = 0;
            R12[2][2] = 1;

            // T23 (alpha=-90): [c3 0 -s3; s3 0 c3; 0 -1 0]
            Matrixf<3, 3> R23;
            R23[0][0] = c3;
            R23[0][1] = 0;
            R23[0][2] = -s3;
            R23[1][0] = s3;
            R23[1][1] = 0;
            R23[1][2] = c3;
            R23[2][0] = 0;
            R23[2][1] = -1;
            R23[2][2] = 0;

            Matrixf<3, 3> R03 = R01 * R12 * R23;

            // 计算 R36 = R03' * R_target
            Matrixf<3, 3> R_target = t2r(T_target);
            Matrixf<3, 3> R36 = R03.trans() * R_target;

            // === 欧拉角提取 (针对 alpha4=90, alpha5=-90) ===
            float r13 = R36[0][2];
            float r23 = R36[1][2];
            float r31 = R36[2][0];
            float r32 = R36[2][1];
            float r33 = R36[2][2];

            // q5 的两个解
            float q5_candidates[2];
            // atan2(sqrt(x^2+y^2), z)
            q5_candidates[0] = atan2f(sqrtf(r13 * r13 + r23 * r23), r33);
            q5_candidates[1] = atan2f(-sqrtf(r13 * r13 + r23 * r23), r33);

            for (int m = 0; m < 2; m++)
            {
                float q5 = normalize_angle(q5_candidates[m]);
                float q4, q6;
                float s5 = sinf(q5);

                if (fabsf(s5) < 1e-4f)
                { // 奇异点
                    q4 = 0.0f;
                    // R11 = c4c6 - s4s6 = c(4+6) -> 这种情况下
                    // 使用 MATLAB 逻辑: q6 = atan2(-r12, r11)
                    q6 = atan2f(-R36[0][1], R36[0][0]);
                }
                else
                {
                    // q4 = atan2(-r23, -r13)
                    q4 = atan2f(-r23 / s5, -r13 / s5);
                    // q6 = atan2(-r32, r31)
                    q6 = atan2f(-r32 / s5, r31 / s5);
                }

                q4 = normalize_angle(q4);
                q6 = normalize_angle(q6);

                // 保存这一组解
                if (sol_count < 8)
                {
                    float sol_data[6] = {q1, q2, q3, q4, q5, q6};
                    solutions[sol_count] = Matrixf<6, 1>(sol_data);
                    sol_count++;
                }
            }
        }
    }
    return sol_count;
}

// 重载版本：直接使用旋转矩阵 R，避开 Euler 角的歧义
uint8_t ikine_pieper_solutions(float pos_target[3], Matrixf<3, 3> R_target, Matrixf<6, 1> solutions[8])
{
    // 1. 直接使用传入的旋转矩阵，不再进行 RPY 转换
    // Matrixf<4, 4> T_rot = rpy2t(rpy); <--- 删除这一步

    // 设置位置向量
    Matrixf<3, 1> pos;
    pos[0][0] = pos_target[0];
    pos[1][0] = pos_target[1];
    pos[2][0] = pos_target[2];

    // 2. 组合 T_target = [R  P]
    //                   [0  1]
    // rp2t 函数应该是支持 (Matrix3x3, Matrix3x1) 的，如果不支持，请手动构建
    Matrixf<4, 4> T_target = rp2t(R_target, pos);

    // ==========================================
    // 下面的代码与原函数完全一致，直接复制下来即可
    // ==========================================

    int sol_count = 0;
    float d1 = 8.0f;
    float a2 = 35.0f;
    float d4 = 27.8f;
    float d6 = 24.5f;
    float offset2 = PI / 2.0f;

    // 求解手腕中心 ... (完全复制原函数剩余部分)
    Matrixf<3, 1> P = t2p(T_target);
    Matrixf<3, 1> A = T_target.block<3, 1>(0, 2);

    Matrixf<3, 1> Pc = P - A * d6;
    float Pcx = Pc[0][0];
    float Pcy = Pc[1][0];
    float Pcz = Pc[2][0];

    // 第一阶段：位置求解 q1, q2, q3

    // q1 的两个候选解 (正向/背向)
    float q1_candidates[2];
    q1_candidates[0] = atan2f(Pcy, Pcx);
    q1_candidates[1] = atan2f(-Pcy, -Pcx);

    for (int i = 0; i < 2; i++)
    {
        float q1 = normalize_angle(q1_candidates[i]);

        // 将手腕中心投影到臂平面
        float r = sqrtf(Pcx * Pcx + Pcy * Pcy);
        if (i == 1)
            r = -r; // 背向解，r 取负

        float s = Pcz - d1;

        // --- 几何参数计算 ---
        float D2 = r * r + s * s;
        float D = sqrtf(D2);

        // 余弦定理求内角 phi (a2 与 d4 的夹角)
        float cos_phi = (a2 * a2 + d4 * d4 - D2) / (2.0f * a2 * d4);

        // 检查是否超出工作空间
        if (fabsf(cos_phi) > 1.0001f)
        {
            continue;
        }
        // 数值误差修正
        if (cos_phi > 1.0f)
            cos_phi = 1.0f;
        if (cos_phi < -1.0f)
            cos_phi = -1.0f;

        float phi = acosf(cos_phi);

        // 余弦定理求仰角偏差 beta (a2 与 D 的夹角)
        float cos_beta = (a2 * a2 + D2 - d4 * d4) / (2.0f * a2 * D);

        if (cos_beta > 1.0f)
            cos_beta = 1.0f;
        if (cos_beta < -1.0f)
            cos_beta = -1.0f;

        float beta = acosf(cos_beta);
        float psi = atan2f(s, r);

        // 构造两组 q2/q3 解 (Elbow Up / Elbow Down)
        float q3_opts[2];
        float q2_geom_opts[2];

        // 解A: 肘部向上
        q3_opts[0] = PI - phi;
        q2_geom_opts[0] = psi - beta;

        // 解B: 肘部向下
        q3_opts[1] = phi - PI;
        q2_geom_opts[1] = psi + beta;

        for (int k = 0; k < 2; k++)
        {
            // 计算实际 q2, q3
            float q3 = normalize_angle(q3_opts[k]);
            float q2 = normalize_angle(q2_geom_opts[k] - offset2);

            // 姿态求解 q4, q5, q6

            // 计算前三轴产生的旋转矩阵 R03
            // R03 = R01 * R12 * R23

            // 预计算三角函数
            float c1 = cosf(q1), s1 = sinf(q1);
            float th2 = q2 + offset2; // R12 用的 theta
            float c2 = cosf(th2), s2 = sinf(th2);
            float th3 = q3; // R23 用的 theta
            float offset3 = -PI / 2.0f;
            float th3_mat = q3 + offset3;
            float c3 = cosf(th3_mat), s3 = sinf(th3_mat);

            // 手动构建矩阵 (比通用矩阵乘法快)
            // T01 (alpha=90): [c1 0 s1; s1 0 -c1; 0 1 0]
            Matrixf<3, 3> R01;
            R01[0][0] = c1;
            R01[0][1] = 0;
            R01[0][2] = s1;
            R01[1][0] = s1;
            R01[1][1] = 0;
            R01[1][2] = -c1;
            R01[2][0] = 0;
            R01[2][1] = 1;
            R01[2][2] = 0;

            // T12 (alpha=0): [c2 -s2 0; s2 c2 0; 0 0 1]
            Matrixf<3, 3> R12;
            R12[0][0] = c2;
            R12[0][1] = -s2;
            R12[0][2] = 0;
            R12[1][0] = s2;
            R12[1][1] = c2;
            R12[1][2] = 0;
            R12[2][0] = 0;
            R12[2][1] = 0;
            R12[2][2] = 1;

            // T23 (alpha=-90): [c3 0 -s3; s3 0 c3; 0 -1 0]
            Matrixf<3, 3> R23;
            R23[0][0] = c3;
            R23[0][1] = 0;
            R23[0][2] = -s3;
            R23[1][0] = s3;
            R23[1][1] = 0;
            R23[1][2] = c3;
            R23[2][0] = 0;
            R23[2][1] = -1;
            R23[2][2] = 0;

            Matrixf<3, 3> R03 = R01 * R12 * R23;

            // 计算 R36 = R03' * R_target
            Matrixf<3, 3> R_target = t2r(T_target);
            Matrixf<3, 3> R36 = R03.trans() * R_target;

            // === 欧拉角提取 (针对 alpha4=90, alpha5=-90) ===
            float r13 = R36[0][2];
            float r23 = R36[1][2];
            float r31 = R36[2][0];
            float r32 = R36[2][1];
            float r33 = R36[2][2];

            // q5 的两个解
            float q5_candidates[2];
            // atan2(sqrt(x^2+y^2), z)
            q5_candidates[0] = atan2f(sqrtf(r13 * r13 + r23 * r23), r33);
            q5_candidates[1] = atan2f(-sqrtf(r13 * r13 + r23 * r23), r33);

            for (int m = 0; m < 2; m++)
            {
                float q5 = normalize_angle(q5_candidates[m]);
                float q4, q6;
                float s5 = sinf(q5);

                if (fabsf(s5) < 1e-4f)
                { // 奇异点
                    q4 = 0.0f;
                    // R11 = c4c6 - s4s6 = c(4+6) -> 这种情况下
                    // 使用 MATLAB 逻辑: q6 = atan2(-r12, r11)
                    q6 = atan2f(-R36[0][1], R36[0][0]);
                }
                else
                {
                    // q4 = atan2(-r23, -r13)
                    q4 = atan2f(-r23 / s5, -r13 / s5);
                    // q6 = atan2(-r32, r31)
                    q6 = atan2f(-r32 / s5, r31 / s5);
                }

                q4 = normalize_angle(q4);
                q6 = normalize_angle(q6);

                // 保存这一组解
                if (sol_count < 8)
                {
                    float sol_data[6] = {q1, q2, q3, q4, q5, q6};
                    solutions[sol_count] = Matrixf<6, 1>(sol_data);
                    sol_count++;
                }
            }
        }
    }

    return sol_count;
}

uint8_t solution_filter(Matrixf<6, 1> solutions[8], bool valid[8])
{
    int valid_count = 0;
    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;
        for (int j = 0; j < 6; j++)
        {
            float angle = solutions[i][j][0];
            // 检查每个关节角度是否在范围内
            if (angle < qmin[j] || angle > qmax[j])
            {
                valid[i] = false;
                break;
            }
        }
        if (valid[i])
        {
            valid_count++;
        }
    }
    return valid_count;
}

uint8_t get_best_solution_index(Matrixf<6, 1> solutions[8], bool valid[8], float current_angle[6])
{
    float d[8] = {0.0f};    // 欧式距离，等于各关节角差值平方之和再求平方根(这里略去求根这一步)
    float err[6] = {0.0f};  // 各个关节的角度差值
    uint8_t best_index = 0; // 求最值经典起手式
    for (int i = 0; i < 8; i++)
    {
        if (!valid[i])
        {
            d[i] = 114514.1919f;
        }

        for (int j = 0; j < 6; j++)
        {
            err[j] = current_angle[j] - solutions[i][j][0];
            err[j] *= err[j];
            d[i] += err[j];
        }

        if (d[i] < d[best_index])
            best_index = i;
    }

    return best_index;
}

// 测试用函数，传入当前的角度值，指定的轴，平移长度和用于存放结果的数组，计算出平移后的xyz坐标，已验证
void move_result(float q[6], uint8_t axis, float s, float pos[3])
{
    float xyz[3]; // 平移前的xyz
    /*计算指定方向的方向向量*/
    float axis_vector[3] = {0.0f};
    // 0 - X-axis / 1 - Y-axis / 2 - Z-axis
    Matrixf<3, 3> R = robot.fkine(Matrixf<6, 1>(q)).block<3, 3>(0, 0);
    axis_vector[0] = R[0][axis];
    axis_vector[1] = R[1][axis];
    axis_vector[2] = R[2][axis];

    // 计算当前xyz
    xyz[0] = robot.fkine(Matrixf<6, 1>(q))[0][3];
    xyz[1] = robot.fkine(Matrixf<6, 1>(q))[1][3];
    xyz[2] = robot.fkine(Matrixf<6, 1>(q))[2][3];

    /*计算平移后的坐标*/
    pos[0] = xyz[0] + s * axis_vector[0];
    pos[1] = xyz[1] + s * axis_vector[1];
    pos[2] = xyz[2] + s * axis_vector[2];
}

/**
 * @brief 默认的梯形速度规划
 * length = 10.0cm, v_max = 4.0cm/s, a_max = 8.0cm/s^2
 * t_acc = 0.5s, t_flat = 2.0s, t_dec = 0.5s, t_total = 3.0s
 *
 * @param t_ms   当前时间 (毫秒)
 */
float get_s_at(uint32_t t_ms)
{
    // ms转成s
    float t_s = t_ms * 0.001f;
    float s = 0.0f, l = 10.0f, a = 8.0f, v = 4.0f;
    // 加速度 a = 8, 匀速运动速度 v = 4
    if (t_s < 0.5f)
    {
        s += 0.5f * a * t_s * t_s; // 加速阶段 s = 0.5 * a * t^2
    }
    else if (t_s < 2.5f)
    {
        s += 0.5f * a * 0.5f * 0.5f; // 加速段位移
        s += v * (t_s - 0.5f);       // 匀速段位移 s = s_acc + v * (t - t_acc)
    }
    else if (t_s <= 3.0f)
    {
        float t_remain = 3.0f - t_s;
        s = l - 0.5f * a * t_remain * t_remain;
    }
    else
    {
        s = l;
    }

    return s;
}

void calculate_trajectory_xyz(float q_start[6], uint8_t axis, float trajectory_xyz[600][3])
// 目前设置的末端移动速度为4cm/s, 加速度a = 8.0cm/s^2， 总长10cm需要运动3.0s，即3000ms，控制器每5ms向电机更新一次数据，一共需要计算3000 / 4 = 600个点
// 采样精度设置为5ms一次，共600个点
{
    static uint32_t cal_cnt = 0;
    float s = get_s_at(cal_cnt * 5);
    move_result(q_start, axis, s, trajectory_xyz[cal_cnt]);
    if (cal_cnt == 600)
        cal_cnt = 0; // 计算完成后清零，防止溢出
    cal_cnt++;
}

uint32_t ikine_trajectory(float trajectory_xyz[600][3], float rpy_target[3], float q_solution[600][6], float q_start[6])
{
    static uint32_t cal_cnt = 0; // 计算的次数，完成第一次计算则为1，一次都没完成则为0，可以看作是函数的调用次数
    uint32_t result_cnt;         // 有效结果的个数0-600
    bool is_traj_valid = true;
    Matrixf<6, 1> solutions[8];
    bool valid_index[8];

    ikine_pieper_solutions(trajectory_xyz[cal_cnt], rpy_target, solutions);
    cal_cnt++;

    uint8_t valid_count = solution_filter(solutions, valid_index);
    if (valid_count == 0)
    {
        is_traj_valid = false;
    }
    else
    {
        result_cnt++;
    }

    if (is_traj_valid)
    {
        if (cal_cnt == 1)
        {
            uint8_t best_index = get_best_solution_index(solutions, valid_index, q_start);
            for (int j = 0; j < 6; j++)
            {
                q_solution[cal_cnt - 1][j] = solutions[best_index][j][0];
            }
        }
        else
        {
            uint8_t best_index = get_best_solution_index(solutions, valid_index, q_solution[cal_cnt - 2]);
            for (int j = 0; j < 6; j++)
            {
                q_solution[cal_cnt - 1][j] = solutions[best_index][j][0];
            }
        }

        if (cal_cnt == 600)
        {
            cal_cnt = 600;
        }
    }

    return result_cnt;
}

// 测试用函数，放回调里，测试get_s_at的计算是否正确
float test_get_s_at()
{
    static uint32_t t_ms = 0;
    if (t_ms > 3000)
    {
        t_ms = 3000;
    }
    return get_s_at(t_ms++);
}