#include "robot_control.h"
#include "small_driver_uart_control.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "../../hmi/ui/page_remote.h"
#include "../navigation/nav_engine.h"
#include "../../control/leg/angle_offset.h"
#include "../../control/leg/leg_pid_control.h"
#include "../../control/leg/kinematics.h"
#include "../../control/leg/jacobian.h"
#include <math.h>

#define M_PI 3.1415926f
#define M_PI_2 (M_PI / 2.0f)

static Foot_position_t foot_position_left;

static Foot_position_t foot_position_right;

/* 腿部关节 PID 控制器 */
static Leg_PID_t g_leg_left_pid, g_leg_right_pid;

/* 腿部关节目标角度 (相对限位的偏移量, rad) */
static Leg_Target_t g_leg_target_left, g_leg_target_right;

static PID_Controller_t g_pitch_angle_pid, g_pitch_gyro_pid, g_speed_pid, g_speed_right_pid;
static PID_Controller_t g_leg_speed_pid, g_leg_roll_pid;
#define LEG_NOM_FRONT_L     0.5f
#define LEG_NOM_BACK_L     -0.5f
#define LEG_NOM_FRONT_R    -0.5f
#define LEG_NOM_BACK_R      0.5f

/* 右腿绝对角度转换符号：镜像安装, 传感器 CW 使绝对角减小 */
#define RIGHT_ABS_FRONT_SIGN  -1.0f
#define RIGHT_ABS_BACK_SIGN   -1.0f

void robot_control_init(void){
    //pitch的PID
    pid_init(&g_pitch_angle_pid, 25.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 270.0f, 0.0f);
    pid_init(&g_pitch_gyro_pid,  -1170.0f, 0.0f, 3.00f, ROBOT_CONTROL_DT, 10000.0f, 3000.0f);
    pid_init(&g_speed_pid,       0.4f, 0.0f, 0.00f, ROBOT_CONTROL_DT, 10000.0f, 500.0f);
    pid_init(&g_speed_right_pid, 15.5f, 0.0f, 0.05f, ROBOT_CONTROL_DT, 10000.0f, 100.0f);
    //针对腿位置的PID，需要在完整的车上调试
    pid_init(&g_leg_speed_pid, 30.0f, 0.5f, 0.0f, ROBOT_CONTROL_DT, 30.0f, 10.0f);
    pid_init(&g_leg_roll_pid,  10.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 1.0f, 0.5f);

    //关节角度的PID，需要在完整的车上调试
    leg_pid_init(&g_leg_left_pid,  1500.0f, 0.0f, 0.0f, 10000.0f, 0.0f);
    leg_pid_init(&g_leg_right_pid, 1500.0f, 0.0f, 0.0f, 10000.0f, 0.0f);

    //初始偏移，根据实际情况改一下重心
    g_leg_target_left.front  = 0.4f;
    g_leg_target_left.back   = -0.4f;
    g_leg_target_right.front = 0.0f;
    g_leg_target_right.back  = 0.0f;

    remote_param_bind(0, &g_pitch_angle_pid.kp);
    remote_param_bind(1, &g_pitch_gyro_pid.kp);
    remote_param_bind(2, &g_pitch_gyro_pid.kd);
    remote_param_bind(3, &g_speed_pid.kp);
    remote_param_bind(4, &g_speed_right_pid.kp);
    remote_param_bind(5, &g_leg_left_pid.front.kp);
    remote_param_bind(6, &g_leg_roll_pid.kp);

}

/*---------------------------------------------------------------------------*/
void sensor_update(const Sensor_data_t *sensor){
    g_sensor_data = *sensor;
}

/*---------------------------------------------------------------------------*/
void command_update(const Move_cmd_t *cmd){
    g_move_cmd = *cmd;
}

/*---------------------------------------------------------------------------*/
void control_task(void){
    Sensor_data_t sensor_local = g_sensor_data;
    Move_cmd_t     cmd_local   = g_move_cmd;

    //pitch平衡环
    pitch_balance_control(&sensor_local, &g_speed_pid, &g_pitch_angle_pid,
        &g_pitch_gyro_pid, &g_motor_cmd);

    //计算目标足端位置
    leg_cmd_solve(&cmd_local, &sensor_local, &g_leg_speed_pid, &g_leg_roll_pid,
        &foot_position_left, &foot_position_right);

    //腿的控制
    /* ---- 左腿 ---- */
    {
        Joint_angle_t abs_nominal;      /* 标称位形对应的绝对角度 */
        float J[2][2], dth1, dth2;

        abs_nominal.left_motor_angle  = (float)M_PI_2 + LEG_NOM_FRONT_L;

        abs_nominal.right_motor_angle = (float)M_PI_2 + LEG_NOM_BACK_L;

        if (five_bar_jacobian(&abs_nominal, J) == 0 &&
            five_bar_jacobian_solve(J, &dth1, &dth2,
                foot_position_left.x, foot_position_left.y) == 0) {
            g_leg_target_left.front  = LEG_NOM_FRONT_L + dth1;
            g_leg_target_left.back   = LEG_NOM_BACK_L  + dth2;
        } else {
            /* 奇异 → 保持标称 */ //AI给的位置过远时可能会奇异，先保持标称位形，后续可以考虑更安全的处理方式
            g_leg_target_left.front  = LEG_NOM_FRONT_L;
            g_leg_target_left.back   = LEG_NOM_BACK_L;
        }
    }

    /* ---- 右腿 ---- *///下面都是AI给的位置，右腿是镜像安装的，传感器方向也可能相反，所以要调整一下绝对角度的标称位形和符号
    {
        Joint_angle_t abs_nominal;
        float J[2][2], dth1, dth2;

        /* 右腿传感器方向可能因镜像安装而与左腿相反, 通过符号宏调节 */
        abs_nominal.left_motor_angle  = (float)M_PI_2 + RIGHT_ABS_FRONT_SIGN * LEG_NOM_FRONT_R;
        abs_nominal.right_motor_angle = (float)M_PI_2 + RIGHT_ABS_BACK_SIGN  * LEG_NOM_BACK_R;

        if (five_bar_jacobian(&abs_nominal, J) == 0 &&
            five_bar_jacobian_solve(J, &dth1, &dth2,
                foot_position_right.x, foot_position_right.y) == 0) {
            /* target = LEG_NOM + RIGHT_SIGN * dθ (与 abs 转换符号一致) */
            g_leg_target_right.front = LEG_NOM_FRONT_R + RIGHT_ABS_FRONT_SIGN * dth1;
            g_leg_target_right.back  = LEG_NOM_BACK_R  + RIGHT_ABS_BACK_SIGN  * dth2;
        } else {
            /* 奇异 → 保持标称 */ //AI给的位置过远时可能会奇异，先保持标称位形，后续可以考虑更安全的处理方式
            g_leg_target_right.front = LEG_NOM_FRONT_R;
            g_leg_target_right.back  = LEG_NOM_BACK_R;
        }
    }
    //PID加发命令
    leg_pid_control(&g_leg_left_pid, &g_leg_target_left,
                    &sensor_local, LEG_LEFT, &g_motor_cmd);

    leg_pid_control(&g_leg_right_pid, &g_leg_target_right,
                    &sensor_local, LEG_RIGHT, &g_motor_cmd);
}

/*---------------------------------------------------------------------------*/
void sensor_cmd_update(const Ctrl_Input_t *ctrl, Sensor_data_t *sensor, Move_cmd_t *cmd){

    /* --- 传感器数据桥接 --- */
    sensor->angle_pitch    = ctrl->body_pitch;
    sensor->angle_roll     = ctrl->body_roll;
    sensor->angle_yaw      = 0.0f;                     /* 无绝对偏航参考 */
    sensor->gyro_pitch     = ctrl->gyro_pitch_rate;
    sensor->gyro_yaw       = ctrl->gyro_yaw_rate;
    sensor->gyro_roll      = ctrl->gyro_roll_rate;
    sensor->accel_z        = 0.0f;
    //多个板子的话要加入两个small_driver_value，具体引脚还没确定
    sensor->motor_left_speed  = (float)small_driver_value.receive_left_speed_data * RPM_TO_RADPS;
    sensor->motor_right_speed = (float)small_driver_value.receive_right_speed_data * RPM_TO_RADPS;

    sensor->joint_left_front_angle  = (float)small_driver_value.receive_left_location_data * DEG_TO_RAD;
    sensor->joint_left_back_angle   = (float)small_driver_value.receive_right_location_data * DEG_TO_RAD;
    sensor->joint_right_front_angle = ctrl->motor_angle_fr;
    sensor->joint_right_back_angle  = ctrl->motor_angle_br;

    sensor->joint_left_front_speed  = ctrl->motor_vel_fl;
    sensor->joint_left_back_speed   = ctrl->motor_vel_bl;
    sensor->joint_right_front_speed = ctrl->motor_vel_fr;
    sensor->joint_right_back_speed  = ctrl->motor_vel_br;
    //应用标定
    if (angle_offset_is_done()) {
        angle_offset_apply_to_sensor(sensor);
    }

    cmd->target_speed  = -0.5f;//ctrl->velocity_cmd;
    cmd->target_roll   = 0.0f;//ctrl->steering_cmd;
    cmd->target_height = 0.0f;
}
