#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdbool.h>

typedef struct {
    float body_pitch;
    float body_roll;
    float gyro_pitch_rate;
    float gyro_yaw_rate;
    float gyro_roll_rate;
    float steering_cmd;
    float velocity_cmd;
    bool on_bridge;

    float motor_angle_fl;
    float motor_angle_bl;
    float motor_angle_fr;
    float motor_angle_br;
    float motor_vel_fl;
    float motor_vel_bl;
    float motor_vel_fr;
    float motor_vel_br;
} Ctrl_Input_t;

typedef struct {
    float max_wheel_torque;
    float max_roll_angle;
    float roll_safe_threshold;
    float max_leg_length_change;
    float max_lateral_offset;
} Safety_Limits_t;

typedef struct {
    float joint_angle_cmd_fl;
    float joint_angle_cmd_bl;
    float joint_angle_cmd_fr;
    float joint_angle_cmd_br;
    float joint_torque_fl;
    float joint_torque_bl;
    float joint_torque_fr;
    float joint_torque_br;
    float wheel_torque_L;
    float wheel_torque_R;
} Ctrl_Output_t;

#endif
