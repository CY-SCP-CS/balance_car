#ifndef TYPES_H
#define TYPES_H

/* 传感器数据 —— 各控制模块的统一输入 */
typedef struct
{
    float accel_z;

    float gyro_yaw;
    float gyro_pitch;

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
#define LEG_JOINT_DISTANCE      50.0f   /* 关节电机间距（mm） */
#define LEG_THIGH               100.0f  /* 大腿长（mm） */
#define LEG_SHANK               100.0f  /* 小腿长（mm） */
#define LEG_LENGTH_STANDARD     20.0f   /* 标准腿长（mm） */
#define LEG_WHEEL_RADIUS        20.0f   /* 轮子半径（mm） */

/* 单位转换常量 */
#define RPM_TO_RADPS            0.104719755f    /* RPM → rad/s: 2π/60 */

/* 通用工具宏 */
#define ROUND(x)                ((int)((x) > 0 ? (x) + 0.5f : (x) - 0.5f))

/* 控制周期 */
#define ROBOT_CONTROL_DT        0.001f

#endif /* TYPES_H */
