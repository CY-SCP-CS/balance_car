#ifndef HMI_PAGE_IMU_DEBUG_H
#define HMI_PAGE_IMU_DEBUG_H

#include "seekfree_assistant_interface.h"

// 通信方式：按实际连接方式修改
// SEEKFREE_ASSISTANT_DEBUG_UART    → USB/调试串口
// SEEKFREE_ASSISTANT_WIRELESS_UART → 无线串口模块
// SEEKFREE_ASSISTANT_BLE6A20       → 蓝牙透传模块
#define DEBUG_DISPLAY_DEVICE    SEEKFREE_ASSISTANT_DEBUG_UART

// 初始化逐飞助手通信接口，在 main 初始化阶段调用一次
void imu_debug_display_init(void);

// 发送 IMU 6 路示波器数据（调用前 IMU 须已初始化）
// 通道分配：
//   Ch1 pitch (deg)   Ch2 roll (deg)    Ch3 yaw (deg)
//   Ch4 gyro_x(deg/s) Ch5 gyro_y(deg/s) Ch6 gyro_z(deg/s)
// 建议调用频率 ≤ 200 Hz，避免串口溢出
void imu_debug_display_update(void);

#endif
