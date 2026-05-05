#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "all_lib.h"
#include "kinematics.h"
#include <stdbool.h>

// ==============================================
// 安全与限幅参数
// ==============================================
typedef struct {
    float max_roll_angle;           // 最大压弯角度 (rad)
    float roll_safe_threshold;      // 侧翻保护阈值 (rad)
    float max_leg_length_change;   // 腿长最大伸缩量 (mm)
    float max_wheel_torque;        // 驱动轮最大扭矩
    float max_lateral_offset;       // 重心偏移限幅 (mm)
    float low_pass_alpha;           // 滤波系数
} Safety_Limits_t;

// ==============================================
// 反馈数据结构
// ==============================================
typedef struct {
    float body_pitch;               // 机身俯仰角 (rad)
    float gyro_pitch_rate;         // 俯仰角速度 (rad/s)
    float body_roll;               // 机身横滚角 (rad)
    float gyro_yaw_rate;           // 偏航角速度 (rad/s)
    float motor_angle_fl;          // 左前电机角度
    float motor_angle_bl;          // 左后电机角度
    float motor_angle_fr;          // 右前电机角度
    float motor_angle_br;          // 右后电机角度
    float motor_vel_fl;            // 左前电机角速度 (rad/s)
    float motor_vel_bl;            // 左后电机角速度 (rad/s)
    float motor_vel_fr;            // 右前电机角速度 (rad/s)
    float motor_vel_br;            // 右后电机角速度 (rad/s)
    float velocity_cmd;            // 速度指令 [-1,1]
    float steering_cmd;            // 转向指令 [-1,1]
    int   on_bridge;               // 单边桥状态
} Feedback_Data_t;

// ==============================================
// 输出数据结构
// ==============================================
typedef struct {
    float wheel_torque_L;          // 左轮转矩
    float wheel_torque_R;          // 右轮转矩
    float joint_angle_cmd_fl;      // 左前关节目标角度
    float joint_angle_cmd_bl;      // 左后关节目标角度
    float joint_angle_cmd_fr;      // 右前关节目标角度
    float joint_angle_cmd_br;      // 右后关节目标角度
    float joint_torque_fl;        // 左前关节力矩
    float joint_torque_bl;        // 左后关节力矩
    float joint_torque_fr;        // 右前关节力矩
    float joint_torque_br;        // 右后关节力矩
} Output_Data_t;


float run_pitch_control(const Feedback_Data_t *fb,
                        const Safety_Limits_t *lim,
                        PID_Controller_t *outer_pid,
                        PID_Controller_t *inner_pid);


float run_roll_control(const Feedback_Data_t *fb,
                       const Safety_Limits_t *lim,
                       PID_Controller_t *pid_roll);


float run_yaw_control(const Feedback_Data_t *fb,
                       const Safety_Limits_t *lim,
                       PID_Controller_t *pid_yaw,
                       float max_yaw_rate);


void run_joint_pd_control(const Output_Data_t *target,
                          const Feedback_Data_t *fb,
                          const Safety_Limits_t *lim,
                          PD_Controller_t *pd_fl, PD_Controller_t *pd_bl,
                          PD_Controller_t *pd_fr, PD_Controller_t *pd_br,
                          Output_Data_t *out);


void run_leg_position_control(float leg_length_delta,
                               const LegConfig_t *leg,
                               const Feedback_Data_t *fb,
                               const Safety_Limits_t *lim,
                               PID_Controller_t *pid_x_L, PID_Controller_t *pid_y_L,
                               PID_Controller_t *pid_x_R, PID_Controller_t *pid_y_R,
                               Output_Data_t *out);


void run_torque_distribution(float total_torque, float steer_diff,
                              const Safety_Limits_t *lim,
                              Output_Data_t *out);

#endif // CONTROLLER_H
