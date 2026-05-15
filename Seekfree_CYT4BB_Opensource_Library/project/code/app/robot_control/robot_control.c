#include "robot_control.h"
#include "small_driver_uart_control.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "../../hmi/ui/page_remote.h"
#include "../navigation/nav_engine.h"



static Foot_position_t foot_position_left;

static Foot_position_t foot_position_right;

static VMC_Config_t vmc_config;

static PID_Controller_t g_pitch_angle_pid, g_pitch_gyro_pid, g_speed_pid, g_speed_right_pid;

void robot_control_init(void){

    pid_init(&g_pitch_angle_pid, 1.5f, 0.0f, 0.8f, ROBOT_CONTROL_DT, 10.0f, 0.0f);
    pid_init(&g_pitch_gyro_pid,  200.0f, 0.0f, 0.10f, ROBOT_CONTROL_DT, 10000.0f, 3000.0f);
    pid_init(&g_speed_pid,  0.005f, 0.0f, 0.00f, ROBOT_CONTROL_DT, 0.25f, 0.1f);
    pid_init(&g_speed_right_pid, 15.5f, 0.0f, 0.05f, ROBOT_CONTROL_DT, 10000.0f, 100.0f);

    vmc_config.kp = 500.0f; 
    vmc_config.kd = 15.0f;   

    remote_param_bind(0, &g_pitch_angle_pid.kp);
    remote_param_bind(1, &g_pitch_gyro_pid.kp);
    remote_param_bind(2, &g_pitch_gyro_pid.kd);
    remote_param_bind(3, &g_speed_pid.kp);
    remote_param_bind(4, &g_speed_right_pid.kp);
    remote_param_bind(5, &vmc_config.kp);
    remote_param_bind(6, &g_pitch_angle_pid.kd);
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

    /* 1. 车体平衡控制 —— 驱动轮 PWM */
    pitch_balance_control(&sensor_local, &g_speed_pid, &g_pitch_angle_pid,
        &g_pitch_gyro_pid, &g_motor_cmd);

    /* 2. 腿部运动规划 —— 足端目标位置 */
    leg_cmd_solve(&cmd_local, &foot_position_left,
        &foot_position_right);

    /* 3. 左腿 VMC */
    {
        Joint_angle_t angles_cur;
        angles_cur.left_motor_angle  = sensor_local.joint_left_front_angle;
        angles_cur.right_motor_angle = sensor_local.joint_left_back_angle;

        vmc_calculate(&vmc_config, &foot_position_left,
            &angles_cur,
            sensor_local.joint_left_front_speed,
            sensor_local.joint_left_back_speed,
            LEG_LEFT, &g_motor_cmd);
    }

    /* 4. 右腿 VMC */
    {
        Joint_angle_t angles_cur;
        angles_cur.left_motor_angle  = sensor_local.joint_right_front_angle;
        angles_cur.right_motor_angle = sensor_local.joint_right_back_angle;

        vmc_calculate(&vmc_config, &foot_position_right,
            &angles_cur,
            sensor_local.joint_right_front_speed,
            sensor_local.joint_right_back_speed,
            LEG_RIGHT, &g_motor_cmd);
    }
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

    sensor->motor_left_speed  = (float)small_driver_value.receive_left_speed_data * RPM_TO_RADPS;
    sensor->motor_right_speed = (float)small_driver_value.receive_right_speed_data * RPM_TO_RADPS;

    /* 关节角度/速度（注意：kinematics 内 sinf/cosf 期望弧度）
        将来从小驱动器 UART 0x04/0x05 反馈填入时须 × DEG_TO_RAD */
    sensor->joint_left_front_angle  = ctrl->motor_angle_fl;
    sensor->joint_left_back_angle   = ctrl->motor_angle_bl;
    sensor->joint_right_front_angle = ctrl->motor_angle_fr;
    sensor->joint_right_back_angle  = ctrl->motor_angle_br;

    sensor->joint_left_front_speed  = ctrl->motor_vel_fl;
    sensor->joint_left_back_speed   = ctrl->motor_vel_bl;
    sensor->joint_right_front_speed = ctrl->motor_vel_fr;
    sensor->joint_right_back_speed  = ctrl->motor_vel_br;

    /* --- 运动指令桥接 --- */
    cmd->target_speed  = 0.0f;//ctrl->velocity_cmd;
    cmd->target_roll   = 0.0f;//ctrl->steering_cmd;
    cmd->target_height = 0.0f;
}
