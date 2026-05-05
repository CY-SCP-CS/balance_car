#include "robot_control.h"
#include <string.h>

LegConfig_t        g_leg;
Safety_Limits_t    g_limits;
Feedback_Data_t    g_fb;
Output_Data_t      g_out;

PID_Controller_t g_pid_pitch_angle, g_pid_pitch_speed, g_pid_roll, g_pid_yaw;
PD_Controller_t  g_pd_joint_fl, g_pd_joint_bl, g_pd_joint_fr, g_pd_joint_br;
PID_Controller_t g_pid_leg_x_L, g_pid_leg_y_L, g_pid_leg_x_R, g_pid_leg_y_R;

void robot_control_init(void) {
    const float DEG2RAD = 3.1415926f / 180.0f;
    memset(&g_fb, 0, sizeof(g_fb));
    memset(&g_out, 0, sizeof(g_out));

    // 机械参数，根据实际标定
    g_leg = (LegConfig_t){
        .l1 = 60.0f,
        .l2 = 90.0f,
        .hip_offset = 19.0f,
        .nominal_leg_length = 40.0f,//尽量给小，根据机械极限确定
        .wheel_radius = 32.5f
    };

    // 安全限幅，根据实际标定
    g_limits = (Safety_Limits_t){
        .max_roll_angle = 15.0f * DEG2RAD,//待定
        .roll_safe_threshold = 25.0f * DEG2RAD,
        .max_leg_length_change = 40.0f,
        .max_wheel_torque = 10000.0,//PWM最大值,这个根据例程设置
        .max_lateral_offset = 30.0f,
        .low_pass_alpha = 0.15f//可能略大
    };

    // ==================== 姿态控制 PID ====================
    // 俯仰角度环 (外环)
    g_pid_pitch_angle = (PID_Controller_t){8.0f, 0.0f, 0.05f, 300.0f, 300.0f};
    // 俯仰角速度环 (内环)
    g_pid_pitch_speed  = (PID_Controller_t){5.0f, 0.05f, 0.02f, 300.0f, 300.0f};
    // 横滚控制
    g_pid_roll        = (PID_Controller_t){15.0f, 0.08f, 0.05f, 300.0f, 300.0f};
    // 偏航控制
    g_pid_yaw         = (PID_Controller_t){4.0f, 0.04f, 0.02f, 150.0f, 150.0f};

    // ==================== 关节角度闭环 PD ====================
    g_pd_joint_fl = (PD_Controller_t){12.0f, 0.4f, 80.0f};
    g_pd_joint_bl = (PD_Controller_t){12.0f, 0.4f, 80.0f};
    g_pd_joint_fr = (PD_Controller_t){12.0f, 0.4f, 80.0f};
    g_pd_joint_br = (PD_Controller_t){12.0f, 0.4f, 80.0f};

    // ==================== 虚拟腿位置闭环 PID ====================
    g_pid_leg_x_L = (PID_Controller_t){3.0f, 0.05f, 0.0f, 100.0f, 100.0f};
    g_pid_leg_y_L = (PID_Controller_t){3.5f, 0.05f, 0.0f, 100.0f, 100.0f};
    g_pid_leg_x_R = (PID_Controller_t){3.0f, 0.05f, 0.0f, 100.0f, 100.0f};
    g_pid_leg_y_R = (PID_Controller_t){3.5f, 0.05f, 0.0f, 100.0f, 100.0f};
}
//AI瞎给的参数，还没调

void update_feedback(const Feedback_Data_t *fb) {
    memcpy(&g_fb, fb, sizeof(g_fb));
}


//控制逻辑，到时候在这里面加入状态机
void control_task(void) {
//pitch平衡
    float total_torque = run_pitch_control(&g_fb, &g_limits,
                                          &g_pid_pitch_angle, &g_pid_pitch_speed);
//roll平衡
    float leg_length_delta = run_roll_control(&g_fb, &g_limits, &g_pid_roll);
//关节电机计算目标角度
    run_leg_position_control(leg_length_delta, &g_leg, &g_fb, &g_limits,
                             &g_pid_leg_x_L, &g_pid_leg_y_L,
                             &g_pid_leg_x_R, &g_pid_leg_y_R,
                             &g_out);
//关节角度闭环
    run_joint_pd_control(&g_out, &g_fb, &g_limits,
                          &g_pd_joint_fl, &g_pd_joint_bl,
                          &g_pd_joint_fr, &g_pd_joint_br,
                          &g_out);

//差速
    float steer_diff = run_yaw_control(&g_fb, &g_limits, &g_pid_yaw, 1.2f);

//叠加
    run_torque_distribution(total_torque, steer_diff, &g_limits, &g_out);
}
