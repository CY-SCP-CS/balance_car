#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_4        )

#define SMALL_DRIVER_BAUDRATE                   (460800        )

#define SMALL_DRIVER_RX                         (UART4_TX_P14_1)

#define SMALL_DRIVER_TX                         (UART4_RX_P14_0)

#define LEFT_MOTOR_DIR                          (1)

#define RIGHT_MOTOR_DIR                         (-1)

typedef struct
{
    uart_index_enum driver_uart;                        
    
    unsigned char send_data_buffer[7];
    unsigned char receive_data_buffer[7];
    unsigned char receive_data_count;
    unsigned char sum_check_data;

    short int left_motor_dir;                           
    short int right_motor_dir;
    
    short int receive_left_speed_data;                  // 接收到的 左侧电机 转速数据
    short int receive_right_speed_data;                 // 接收到的 右侧电机 转速数据
    
    float receive_left_angle_data;                      // 接收到的 左侧电机 角度数据
    float receive_right_angle_data;                     // 接收到的 右侧电机 角度数据
    
    float receive_left_location_data;                   // 接收到的 左侧电机 位置数据
    float receive_right_location_data;                  // 接收到的 右侧电机 位置数据
}small_device_value_struct;

extern small_device_value_struct small_driver_value;


void small_driver_control_callback(small_device_value_struct *driver_value);                            // 无刷驱动 串口接收回调函数

void small_driver_set_duty(small_device_value_struct *driver_value, int left_duty, int right_duty);     // 无刷驱动 设置 电机占空比
    
void small_driver_set_location_zero(small_device_value_struct *driver_value);                           // 无刷驱动 设置 零位置

void small_driver_get_speed(small_device_value_struct *driver_value);                                   // 无刷驱动 获取 转速数据

void small_driver_get_angle(small_device_value_struct *driver_value);                                   // 无刷驱动 获取 电机当前转子机械角度

void small_driver_get_location(small_device_value_struct *driver_value);                                // 无刷驱动 获取 电机当前通过减速结构后的输出角度 

void small_driver_uart_init(void);                                                                      // 无刷驱动 串口通讯初始化

#endif
