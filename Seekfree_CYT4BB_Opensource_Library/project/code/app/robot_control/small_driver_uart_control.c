#include "small_driver_uart_control.h"

small_device_value_struct small_driver_value;      // 定义通讯参数结构体


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口接收回调函数
// 参数说明     driver_value   驱动对应的数据结构体
// 返回参数     void
// 使用示例     uart_control_callback(&small_driver_value);
// 备注信息     用于解析接收到的速度数据  该函数需要在对应的串口接收中断中调用
//-------------------------------------------------------------------------------------------------------------------
void small_driver_control_callback(small_device_value_struct *driver_value)
{
    uint8 receive_data[7];
    uint8 receive_len;
    
    receive_len = uart_query_buffer(driver_value->driver_uart, receive_data);
        
    if(receive_len)
    {
        if(driver_value->receive_data_buffer[0] != 0xA5)
        {
            driver_value->receive_data_count = 0;
        }
        
        for(int i = 0; i < receive_len; i ++)
        {    
            driver_value->receive_data_buffer[driver_value->receive_data_count ++] = receive_data[i];
        }
        
        if(driver_value->receive_data_count >= 7)
        {
            if(driver_value->receive_data_buffer[0] == 0xA5)
            {
                driver_value->sum_check_data =  driver_value->receive_data_buffer[0] +
                                                driver_value->receive_data_buffer[1] +
                                                driver_value->receive_data_buffer[2] +
                                                driver_value->receive_data_buffer[3] +
                                                driver_value->receive_data_buffer[4] +
                                                driver_value->receive_data_buffer[5];
                if(driver_value->sum_check_data == driver_value->receive_data_buffer[6])
                {
                    if(driver_value->receive_data_buffer[1] == 0x02)
                    {
                        driver_value->receive_left_speed_data = (((int)driver_value->receive_data_buffer[2] << 8) | (int)driver_value->receive_data_buffer[3]);
                        driver_value->receive_right_speed_data = (((int)driver_value->receive_data_buffer[4] << 8) | (int)driver_value->receive_data_buffer[5]);
                    }
                    else if(driver_value->receive_data_buffer[1] == 0x04)
                    {
                        short int left_angle_temp  = (((int)driver_value->receive_data_buffer[2] << 8) | (int)driver_value->receive_data_buffer[3]);
                        short int right_angle_temp = (((int)driver_value->receive_data_buffer[4] << 8) | (int)driver_value->receive_data_buffer[5]);
                        
                        driver_value->receive_left_angle_data  = (float)left_angle_temp  / 100.0f;
                        driver_value->receive_right_angle_data = (float)right_angle_temp / 100.0f;
                    }
                    else if(driver_value->receive_data_buffer[1] == 0x05)
                    {
                        short int left_location_temp  = (((int)driver_value->receive_data_buffer[2] << 8) | (int)driver_value->receive_data_buffer[3]);
                        short int right_location_temp = (((int)driver_value->receive_data_buffer[4] << 8) | (int)driver_value->receive_data_buffer[5]);
                        
                        driver_value->receive_left_location_data  = (float)left_location_temp  / 100.0f * driver_value->left_motor_dir;
                        driver_value->receive_right_location_data = (float)right_location_temp / 100.0f * driver_value->right_motor_dir;
                    }
                    driver_value->receive_data_count = 0;
                    memset(driver_value->receive_data_buffer, 0, sizeof(driver_value->receive_data_buffer));
                }
                else
                {
                    driver_value->receive_data_count = 0;
                    memset(driver_value->receive_data_buffer, 0, sizeof(driver_value->receive_data_buffer));
                }
            }
            else
            {
                driver_value->receive_data_count = 0;
                memset(driver_value->receive_data_buffer, 0, sizeof(driver_value->receive_data_buffer));
            }
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置 电机占空比
// 参数说明     driver_value   驱动对应的数据结构体
// 参数说明     left_duty      左侧电机占空比 数值-10000~10000 对应电机占空比 0% ~ 100%（负数代表电机反转）
// 参数说明     right_duty     右侧电机占空比 数值-10000~10000 对应电机占空比 0% ~ 100%（负数代表电机反转）
// 返回参数     void
// 使用示例     small_driver_set_duty(&small_driver_value, -1000, 1000);
// 备注信息     用于控制电机输出力矩
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(small_device_value_struct *driver_value, int left_duty, int right_duty)
{
    left_duty  = func_limit_ab(left_duty, -10000, 10000);
    right_duty = func_limit_ab(right_duty, -10000, 10000);
    
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0x01;
    driver_value->send_data_buffer[2] = (unsigned char)((left_duty & 0xff00) >> 8);
    driver_value->send_data_buffer[3] = (unsigned char)(left_duty & 0x00ff);
    driver_value->send_data_buffer[4] = (unsigned char)((right_duty & 0xff00) >> 8);
    driver_value->send_data_buffer[5] = (unsigned char)(right_duty & 0x00ff);
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];

    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 设置 零位置
// 参数说明     driver_value   驱动对应的数据结构体
// 返回参数     void
// 使用示例     small_driver_set_location_zero(&small_driver_value);
// 备注信息     用于将减速后输出的当前位置设置为0角度 后续基于此角度旋转
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_location_zero(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0X06;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取 转速数据
// 参数说明     driver_value   驱动对应的数据结构体
// 返回参数     void
// 使用示例     small_driver_get_speed(&small_driver_value);
// 备注信息     获取电机当前转速数据 单位 RPM
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0x02;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取 电机当前转子机械角度
// 参数说明     driver_value   驱动对应的数据结构体
// 返回参数     void
// 使用示例     small_driver_get_angle(&small_driver_value);
// 备注信息     获取电机当前转子机械角度 单位 度
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_angle(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0X04;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 获取 电机当前通过减速结构后的输出角度 
// 参数说明     driver_value   驱动对应的数据结构体
// 返回参数     void
// 使用示例     small_driver_get_location(&small_driver_value);
// 备注信息     获取电机当前通过减速结构后的输出角度  单位 度
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_location(small_device_value_struct *driver_value)
{
    driver_value->send_data_buffer[0] = 0xA5;
    driver_value->send_data_buffer[1] = 0X05;
    driver_value->send_data_buffer[2] = 0x00;
    driver_value->send_data_buffer[3] = 0x00;
    driver_value->send_data_buffer[4] = 0x00;
    driver_value->send_data_buffer[5] = 0x00;
    driver_value->send_data_buffer[6] = driver_value->send_data_buffer[0] +
                                        driver_value->send_data_buffer[1] +
                                        driver_value->send_data_buffer[2] +
                                        driver_value->send_data_buffer[3] +
                                        driver_value->send_data_buffer[4] +
                                        driver_value->send_data_buffer[5];
    uart_write_buffer(driver_value->driver_uart, driver_value->send_data_buffer, 7);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 串口通讯初始化
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    memset(&small_driver_value, 0, sizeof(small_driver_value));                                 // 清除结构体数据
    
    small_driver_value.driver_uart = SMALL_DRIVER_UART;                                         // 记录当前驱动使用的串口号

    small_driver_value.left_motor_dir  = LEFT_MOTOR_DIR;                                        // 记录当前驱动左侧电机的旋转方向
    
    small_driver_value.right_motor_dir = RIGHT_MOTOR_DIR;                                       // 记录当前驱动右侧电机的旋转方向
    
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化
    
    uart_rx_trigger_interrupt(SMALL_DRIVER_UART, 6);                                            // 配置为7字节数组接收
    
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_set_duty(&small_driver_value, 0, 0);                                           // 设置0占空比

    small_driver_get_speed(&small_driver_value);                                                // 获取实时速度数据
}














