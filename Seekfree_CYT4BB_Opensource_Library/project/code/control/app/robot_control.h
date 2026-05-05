#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "kinematics.h"
#include "controller.h"


extern LegConfig_t        g_leg;
extern Safety_Limits_t   g_limits;
extern Feedback_Data_t   g_fb;
extern Output_Data_t     g_out;

//整体PID
extern PID_Controller_t g_pid_pitch_angle;
extern PID_Controller_t g_pid_pitch_speed;
extern PID_Controller_t g_pid_roll;
extern PID_Controller_t g_pid_yaw;

//关节电机
extern PD_Controller_t g_pd_joint_fl;// 左前
extern PD_Controller_t g_pd_joint_bl;
extern PD_Controller_t g_pd_joint_fr;
extern PD_Controller_t g_pd_joint_br;// 右后

//足端闭环
extern PID_Controller_t g_pid_leg_x_L;// 左腿x位置
extern PID_Controller_t g_pid_leg_y_L;
extern PID_Controller_t g_pid_leg_x_R;
extern PID_Controller_t g_pid_leg_y_R;// 右腿y位置


void control_task(void);

void robot_control_init(void);

void update_feedback(const Feedback_Data_t *fb);//直接找例程的结构体+陀螺仪数据

#endif // ROBOT_CONTROL_H
