#include "small_driver_uart_control.h"

small_device_value_struct small_driver_value;               /* 驱动轮电机 (UART4) */
small_device_value_struct small_driver_value_leg_left;      /* 左腿关节电机 (UART6) */
small_device_value_struct small_driver_value_leg_right;     /* 右腿关节电机 (UART3) */


//-------------------------------------------------------------------------------------------------------------------
// 函数名称     无刷通讯 接收回调函数
// 函数说明     driver_value   指向对应的数据结构体
// 返回值       void
// 使用示例     uart_control_callback(&small_driver_value);
// 备注信息     用于接收返回的速度信息  该函数需要在对应的串口接收中断中调用
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
// 函数名称     无刷通讯 发送 占空比
// 函数说明     driver_value   指向对应的数据结构体
// 参数说明     left_duty      左侧占空比 数值-10000~10000 对应占空比 0% ~ 100% 负数代表反方向转动
// 参数说明     right_duty     右侧占空比 数值-10000~10000 对应占空比 0% ~ 100% 负数代表反方向转动
// 返回值       void
// 使用示例     small_driver_set_duty(&small_driver_value, -1000, 1000);
// 备注信息     用于控制电机转动
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
// 函数名称     无刷通讯 设置 零位
// 函数说明     driver_value   指向对应的数据结构体
// 返回值       void
// 使用示例     small_driver_set_location_zero(&small_driver_value);
// 备注信息     用于减速器后的当前位置设置为0角度 电机在此角度上转动
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
// 函数名称     无刷通讯 获取 转速信息
// 函数说明     driver_value   指向对应的数据结构体
// 返回值       void
// 使用示例     small_driver_get_speed(&small_driver_value);
// 备注信息     获取电机当前转速信息 单位 RPM
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
// 函数名称     无刷通讯 获取 电机当前转子机械角度
// 函数说明     driver_value   指向对应的数据结构体
// 返回值       void
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
// 函数名称     无刷通讯 获取 电机当前通过减速结构后的角度
// 函数说明     driver_value   指向对应的数据结构体
// 返回值       void
// 使用示例     small_driver_get_location(&small_driver_value);
// 备注信息     获取电机当前通过减速结构后的角度  单位 度
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
// 函数名称     无刷通讯初始化
// 函数说明     初始化三个驱动板：UART4(驱动轮)、UART6(左腿关节)、UART3(右腿关节)
// 返回值       void
// 使用示例     small_driver_uart_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    /* ── 驱动轮电机 (UART4) ── */
    memset(&small_driver_value, 0, sizeof(small_driver_value));
    small_driver_value.driver_uart       = SMALL_DRIVER_UART_DRIVE;
    small_driver_value.left_motor_dir    = LEFT_MOTOR_DIR;
    small_driver_value.right_motor_dir   = RIGHT_MOTOR_DIR;

    uart_init(SMALL_DRIVER_UART_DRIVE, SMALL_DRIVER_DRIVE_BAUDRATE, SMALL_DRIVER_DRIVE_TX, SMALL_DRIVER_DRIVE_RX);
    uart_rx_trigger_interrupt(SMALL_DRIVER_UART_DRIVE, 6);
    uart_rx_interrupt(SMALL_DRIVER_UART_DRIVE, 1);

    /* ── 左腿关节电机 (UART6) ── */
    memset(&small_driver_value_leg_left, 0, sizeof(small_driver_value_leg_left));
    small_driver_value_leg_left.driver_uart       = SMALL_DRIVER_UART_LEG_LEFT;
    small_driver_value_leg_left.left_motor_dir    = 1;       /* 左前关节方向 */
    small_driver_value_leg_left.right_motor_dir   = 1;       /* 左后关节方向 */

    uart_init(SMALL_DRIVER_UART_LEG_LEFT, SMALL_DRIVER_LEG_LEFT_BAUDRATE, SMALL_DRIVER_LEG_LEFT_TX, SMALL_DRIVER_LEG_LEFT_RX);
    uart_rx_trigger_interrupt(SMALL_DRIVER_UART_LEG_LEFT, 6);
    uart_rx_interrupt(SMALL_DRIVER_UART_LEG_LEFT, 1);

    /* ── 右腿关节电机 (UART3) ── */
    memset(&small_driver_value_leg_right, 0, sizeof(small_driver_value_leg_right));
    small_driver_value_leg_right.driver_uart       = SMALL_DRIVER_UART_LEG_RIGHT;
    small_driver_value_leg_right.left_motor_dir    = 1;       /* 右前关节方向（与左前一致） */
    small_driver_value_leg_right.right_motor_dir   = 1;       /* 右后关节方向 */

    uart_init(SMALL_DRIVER_UART_LEG_RIGHT, SMALL_DRIVER_LEG_RIGHT_BAUDRATE, SMALL_DRIVER_LEG_RIGHT_TX, SMALL_DRIVER_LEG_RIGHT_RX);
    uart_rx_trigger_interrupt(SMALL_DRIVER_UART_LEG_RIGHT, 6);
    uart_rx_interrupt(SMALL_DRIVER_UART_LEG_RIGHT, 1);

    /* ── 发送初始命令 ── */
    small_driver_set_duty(&small_driver_value, 0, 0);               /* 驱动轮停止 */
    small_driver_set_duty(&small_driver_value_leg_left, 0, 0);      /* 左腿关节停止 */
    small_driver_set_duty(&small_driver_value_leg_right, 0, 0);     /* 右腿关节停止 */

    /* 各驱动板只需查询一次，后续会持续回传数据 */
    small_driver_get_speed(&small_driver_value);
    small_driver_get_speed(&small_driver_value_leg_left);
    small_driver_get_speed(&small_driver_value_leg_right);

    small_driver_get_location(&small_driver_value);
    small_driver_get_location(&small_driver_value_leg_left);
    small_driver_get_location(&small_driver_value_leg_right);
}
