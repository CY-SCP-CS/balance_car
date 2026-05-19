#include "robot_control.h"
#include "small_driver_uart_control.h"
#include "../../lib/pid/pid_calculate.h"
#include "../../common/types.h"
#include "../../hmi/ui/page_remote.h"
#include "../navigation/nav_engine.h"
#include "../../control/leg/angle_offset.h"
#include "../../control/leg/kinematics.h"

#define M_PI 3.1415926f

static Foot_position_t foot_position_left;

static Foot_position_t foot_position_right;

static VMC_Config_t vmc_config;

static PID_Controller_t g_pitch_angle_pid, g_pitch_gyro_pid, g_speed_pid, g_speed_right_pid;
static PID_Controller_t g_leg_speed_pid, g_leg_roll_pid;

void robot_control_init(void){

    pid_init(&g_pitch_angle_pid, 25.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 270.0f, 0.0f);
    pid_init(&g_pitch_gyro_pid,  -1170.0f, 0.0f, 3.00f, ROBOT_CONTROL_DT, 10000.0f, 3000.0f);
    pid_init(&g_speed_pid,       0.4f, 0.0f, 0.00f, ROBOT_CONTROL_DT, 10000.0f, 500.0f);
    pid_init(&g_speed_right_pid, 15.5f, 0.0f, 0.05f, ROBOT_CONTROL_DT, 10000.0f, 100.0f);

    /* 腿部速度/横滚闭环 PID（初始值，需现场调试） */
    pid_init(&g_leg_speed_pid, 30.0f, 0.5f, 0.0f, ROBOT_CONTROL_DT, 30.0f, 10.0f);
    pid_init(&g_leg_roll_pid,   1.0f, 0.0f, 0.0f, ROBOT_CONTROL_DT, 1.0f, 0.5f);

    vmc_config.kp = 0.14f;
    vmc_config.kd = 0.0005f;   

    remote_param_bind(0, &g_pitch_angle_pid.kp);
    remote_param_bind(1, &g_pitch_gyro_pid.kp);
    remote_param_bind(2, &g_pitch_gyro_pid.kd);
    remote_param_bind(3, &g_speed_pid.kp);
    remote_param_bind(4, &g_speed_right_pid.kp);
    remote_param_bind(5, &vmc_config.kp);

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

    static Foot_position_t left_zero_angle = {0}, right_zero_angle = {0};
    static bool zero_angle_set_complete = false;
    if (!zero_angle_set_complete) {
        Joint_angle_t joint_zero_angle;
        joint_zero_angle.left_motor_angle  = sensor_local.joint_left_front_angle;
        joint_zero_angle.right_motor_angle = -sensor_local.joint_left_back_angle;
        five_bar_forward(&joint_zero_angle, &left_zero_angle);

        joint_zero_angle.left_motor_angle  = sensor_local.joint_right_front_angle;
        joint_zero_angle.right_motor_angle = sensor_local.joint_right_back_angle;
        five_bar_forward(&joint_zero_angle, &right_zero_angle);

        zero_angle_set_complete = true;
    }

    /* 1. 车体平衡控制 —— 驱动轮 PWM */
    pitch_balance_control(&sensor_local, &g_speed_pid, &g_pitch_angle_pid,
        &g_pitch_gyro_pid, &g_motor_cmd);

    /* 2. 腿部运动规划 —— 足端目标位置（相对于基准的偏移量）
     *    通过 leg_cmd_solve 闭环调节 x(速度) 和 y(横滚) */
    leg_cmd_solve(&cmd_local, &sensor_local, &g_leg_speed_pid, &g_leg_roll_pid,
        &foot_position_left, &foot_position_right);

    /* ── 3. 关节角度→速度（对 1ms 周期差分）
        注意：left_back 取反以补偿 RIGHT_MOTOR_DIR = -1 的反相  ── */
    static float prev_lf = 0, prev_lb = 0, prev_rf = 0, prev_rb = 0;

    float lf_phys = sensor_local.joint_left_front_angle;
    float lb_phys = -sensor_local.joint_left_back_angle;

    float lf_speed = (lf_phys - prev_lf) / ROBOT_CONTROL_DT;
    float lb_speed = (lb_phys - prev_lb) / ROBOT_CONTROL_DT;
    float rf_speed = (sensor_local.joint_right_front_angle - prev_rf) / ROBOT_CONTROL_DT;
    float rb_speed = (sensor_local.joint_right_back_angle  - prev_rb) / ROBOT_CONTROL_DT;

    prev_lf = lf_phys;
    prev_lb = lb_phys;
    prev_rf = sensor_local.joint_right_front_angle;
    prev_rb = sensor_local.joint_right_back_angle;

    /* 4. 左腿 VMC — 目标 = 基准足端 + 命令偏移 */
    {
        Joint_angle_t angles_cur;
        angles_cur.left_motor_angle  = sensor_local.joint_left_front_angle;
        angles_cur.right_motor_angle = -sensor_local.joint_left_back_angle;   /* 补偿 RIGHT_MOTOR_DIR = -1 */

        Foot_position_t left_target;
        left_target.x = 0.0f;       /* 目标居中：足端位于两电机正下方 */
        left_target.y = 200.0f;     /* 目标向下伸展至 y = 200mm */

        vmc_calculate(&vmc_config, &left_target,
            &angles_cur,
            lf_speed,
            lb_speed,
            LEG_LEFT, &g_motor_cmd);
    }

    /* 5. 右腿 VMC — 目标 = 基准足端 + 命令偏移 */
    {
        Joint_angle_t angles_cur;
        angles_cur.left_motor_angle  = sensor_local.joint_right_front_angle;
        angles_cur.right_motor_angle = sensor_local.joint_right_back_angle;

        Foot_position_t right_target;
        right_target.x = right_zero_angle.x + foot_position_right.x;
        right_target.y = right_zero_angle.y + foot_position_right.y;

        vmc_calculate(&vmc_config, &right_target,
            &angles_cur,
            rf_speed,
            rb_speed,
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
        将来驱动器 UART 0x04/0x05 反馈填入时须 × DEG_TO_RAD */
    sensor->joint_left_front_angle  = (float)small_driver_value.receive_left_location_data * DEG_TO_RAD;
    sensor->joint_left_back_angle   = (float)small_driver_value.receive_right_location_data * DEG_TO_RAD;
    sensor->joint_right_front_angle = ctrl->motor_angle_fr;
    sensor->joint_right_back_angle  = ctrl->motor_angle_br;

    sensor->joint_left_front_speed  = ctrl->motor_vel_fl;
    sensor->joint_left_back_speed   = ctrl->motor_vel_bl;
    sensor->joint_right_front_speed = ctrl->motor_vel_fr;
    sensor->joint_right_back_speed  = ctrl->motor_vel_br;

    /* --- 应用标定偏移（标定完成后，将原始编码器角度转为相对限位的角度） --- */
    if (angle_offset_is_done()) {
        angle_offset_apply_to_sensor(sensor);
    }

    /* --- 运动指令桥接 --- */
    cmd->target_speed  = 0.0f;//ctrl->velocity_cmd;
    cmd->target_roll   = 0.0f;//ctrl->steering_cmd;
    cmd->target_height = 0.0f;
}
