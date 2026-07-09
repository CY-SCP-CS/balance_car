#include "page_imu_debug.h"

void page_imu_debug_update(const UI_Frame_t *frame)
{
    const Ctrl_Input_t *fb = frame->fb;

    ui_scope6_send(
        fb != NULL ? fb->body_pitch * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->body_roll * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->body_yaw * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->gyro_pitch_rate * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->gyro_roll_rate * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->gyro_yaw_rate * UI_RAD_TO_DEG : 0.0f);
}
