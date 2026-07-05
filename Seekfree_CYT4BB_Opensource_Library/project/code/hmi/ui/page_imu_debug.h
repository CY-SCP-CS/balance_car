#ifndef HMI_PAGE_IMU_DEBUG_H
#define HMI_PAGE_IMU_DEBUG_H

#include "ui_common.h"

// 发送 IMU 6 路示波器数据（调用前 IMU 须已初始化）
// 通道分配：
//   Ch1 pitch (deg)   Ch2 roll (deg)    Ch3 yaw (deg)
//   Ch4 gyro_x(deg/s) Ch5 gyro_y(deg/s) Ch6 gyro_z(deg/s)
// 建议调用频率 ≤ 200 Hz，避免串口溢出
void page_imu_debug_update(const UI_Frame_t *frame);

#endif
