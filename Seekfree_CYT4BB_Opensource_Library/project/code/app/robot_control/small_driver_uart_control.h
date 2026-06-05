#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


/* ── 驱动轮电机 (UART4) ── */
#define SMALL_DRIVER_UART_DRIVE                 (UART_4        )
#define SMALL_DRIVER_DRIVE_BAUDRATE             (460800        )
#define SMALL_DRIVER_DRIVE_TX                   (UART4_TX_P14_1)   /* MCU TX  → 驱动板 RX */
#define SMALL_DRIVER_DRIVE_RX                   (UART4_RX_P14_0)   /* MCU RX  ← 驱动板 TX */

#define LEFT_MOTOR_DIR                          (1)
#define RIGHT_MOTOR_DIR                         (-1)

/* ── 左腿关节电机 (UART6) ── */
#define SMALL_DRIVER_UART_LEG_LEFT              (UART_6        )
#define SMALL_DRIVER_LEG_LEFT_BAUDRATE          (460800        )
#define SMALL_DRIVER_LEG_LEFT_TX                (UART6_TX_P03_1)   /* MCU TX  → 左腿驱动板 RX */
#define SMALL_DRIVER_LEG_LEFT_RX                (UART6_RX_P03_0)   /* MCU RX  ← 左腿驱动板 TX */

/* ── 右腿关节电机 (UART3) ── */
#define SMALL_DRIVER_UART_LEG_RIGHT             (UART_3        )
#define SMALL_DRIVER_LEG_RIGHT_BAUDRATE         (460800        )
#define SMALL_DRIVER_LEG_RIGHT_TX               (UART3_TX_P13_1)   /* MCU TX  → 右腿驱动板 RX */
#define SMALL_DRIVER_LEG_RIGHT_RX               (UART3_RX_P13_0)   /* MCU RX  ← 右腿驱动板 TX */

typedef struct
{
    uart_index_enum driver_uart;                        
    
    unsigned char send_data_buffer[7];
    unsigned char receive_data_buffer[7];
    unsigned char receive_data_count;
    unsigned char sum_check_data;

    short int left_motor_dir;                           
    short int right_motor_dir;
    
    short int receive_left_speed_data;                  // ���յ��� ����� ת������
    short int receive_right_speed_data;                 // ���յ��� �Ҳ��� ת������
    
    float receive_left_angle_data;                      // ���յ��� ����� �Ƕ�����
    float receive_right_angle_data;                     // ���յ��� �Ҳ��� �Ƕ�����
    
    float receive_left_location_data;                   // ���յ��� ����� λ������
    float receive_right_location_data;                  // ���յ��� �Ҳ��� λ������
}small_device_value_struct;

extern small_device_value_struct small_driver_value;               /* 驱动轮电机 (UART4) */
extern small_device_value_struct small_driver_value_leg_left;      /* 左腿关节电机 (UART6) */
extern small_device_value_struct small_driver_value_leg_right;     /* 右腿关节电机 (UART3) */


void small_driver_control_callback(small_device_value_struct *driver_value);                            // ��ˢ���� ���ڽ��ջص�����

void small_driver_set_duty(small_device_value_struct *driver_value, int left_duty, int right_duty);     // ��ˢ���� ���� ���ռ�ձ�
    
void small_driver_set_location_zero(small_device_value_struct *driver_value);                           // ��ˢ���� ���� ��λ��

void small_driver_get_speed(small_device_value_struct *driver_value);                                   // ��ˢ���� ��ȡ ת������

void small_driver_get_angle(small_device_value_struct *driver_value);                                   // ��ˢ���� ��ȡ �����ǰת�ӻ�е�Ƕ�

void small_driver_get_location(small_device_value_struct *driver_value);                                // ��ˢ���� ��ȡ �����ǰͨ�����ٽṹ�������Ƕ� 

void small_driver_uart_init(void);                                                                      // ��ˢ���� ����ͨѶ��ʼ��

#endif
