#ifndef TYPES_H
#define TYPES_H

#include "../../common/types.h"

/* 传感器数据 —— 各控制模块的统一输入 */
typedef struct
{
    float accel_z;

    float gyro_yaw;
    float gyro_pitch;
    float gyro_roll;

    float angle_pitch;
    float angle_yaw;
    float angle_roll;

    float motor_left_speed;
    float motor_right_speed;

    float joint_left_front_angle;
    float joint_left_back_angle;
    float joint_right_front_angle;
    float joint_right_back_angle;

    float joint_left_front_speed;
    float joint_left_back_speed;
    float joint_right_front_speed;
    float joint_right_back_speed;
} Sensor_data_t;

/* 电机指令 —— 各控制模块的统一输出 */
typedef struct
{
    int left_motor_pwm;
    int right_motor_pwm;
    int left_front_joint_pwm;
    int left_back_joint_pwm;
    int right_front_joint_pwm;
    int right_back_joint_pwm;
} Motor_cmd_duty_t;

typedef struct
{
    float target_speed;     /* 速度    [-1, +1] */
    float target_height;    /* AI给的，暂时没用，保留 [-1, +1] */
    float target_roll;      /* 压弯    [-1, +1] */
} Move_cmd_t;

/* 足端笛卡尔坐标 */
typedef struct
{
    float x;
    float y;
} Foot_position_t;

/* 腿标识 —— 用于 VMC 等需要区分左右腿的接口 */
typedef enum
{
    LEG_LEFT,
    LEG_RIGHT
} LegSide_t;

typedef struct
{
    float kp;  /* 虚拟刚度 */
    float kd;  /* 虚拟阻尼 */
} VMC_Config_t;

/* 五杆机构几何参数 */
#define LEG_JOINT_DISTANCE      48.0f   /* 关节电机间距（mm） */
#define LEG_THIGH               85.0f  /* 大腿长（mm） */
#define LEG_SHANK               135.0f  /* 小腿长（mm） */
#define LEG_LENGTH_STANDARD     45.0f   /* 标准腿长（mm） */
#define LEG_WHEEL_RADIUS        32.5f   /* 轮子半径（mm） */

/* 物理参数 —— 用于 LQR 增益调度 */
#define ROBOT_BODY_MASS         2.5f    /* kg - 车体质量（不含轮） */
#define ROBOT_WHEEL_MASS        0.1f    /* kg - 单个轮质量 */
#define ROBOT_BODY_INERTIA      0.015f  /* kg*m^2 - 车体俯仰惯量 */
#define ROBOT_WHEEL_INERTIA     0.0001f /* kg*m^2 - 轮惯量 */
#define ROBOT_COM_OFFSET        0.025f  /* m - 髋关节到 COM 垂直距离 */
#define ROBOT_GRAVITY           9.81f   /* m/s^2 */
#define LEG_LENGTH_MIN_MM       50.0f   /* mm */
#define LEG_LENGTH_MAX_MM       220.0f  /* mm */

/* 单位转换常量 */
#define RPM_TO_RADPS            0.104719755f    /* RPM → rad/s: 2π/60 */

/* 通用工具宏 */
#define ROUND(x)                ((int)((x) > 0 ? (x) + 0.5f : (x) - 0.5f))

/* 控制周期 */
#define ROBOT_CONTROL_DT        0.001f

#endif /* TYPES_H */
