#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdbool.h>

/* ─── 数学常量 ─────────────────────────────────────────────────── */
#define M_PI        3.141592653589793f
#define M_PI_2      (M_PI / 2.0f)
#define DEG_TO_RAD  (M_PI / 180.0f)
#define RAD_TO_DEG  (180.0f / M_PI)

/* ─── 通用工具宏 ────────────────────────────────────────────────── */
#define CLAMP(val, lo, hi)  (((val) < (lo)) ? (lo) : (((val) > (hi)) ? (hi) : (val)))

/* ─── 驱动轮方向 ────────────────────────────────────────────────── */
#ifndef RIGHT_MOTOR_DIR
#define RIGHT_MOTOR_DIR     (-1)
#endif

/* ─── 腿部标称位形偏置 (rad, 相对限位的偏移) ───────────────────── */
#define LEG_NOM_FRONT_L         1.0f
#define LEG_NOM_BACK_L         -1.0f
#define LEG_NOM_FRONT_R         1.0f
#define LEG_NOM_BACK_R         -1.0f

/* 右腿镜像安装符号补偿 */
#define RIGHT_ABS_FRONT_SIGN    1.0f
#define RIGHT_ABS_BACK_SIGN     1.0f

/* ─── 平衡控制关键参数 ──────────────────────────────────────────── */
#define PITCH_ANGLE_OFFSET_DEG  3.2f    /* IMU 安装偏置 (°), 车体水平时 angle_roll 的读数 */
#define GRAVITY_COMP_GAIN       50.0f   /* 重力补偿前馈增益 */

/* ─── 结构体定义 ────────────────────────────────────────────────── */
typedef struct {
    float body_pitch;
    float body_roll;
    float body_yaw;
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
