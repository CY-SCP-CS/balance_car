#include "zf_common_headfile.h"
#include "../code/sensors/imu/imu.h"
#include "../code/common/types.h"
#include "../code/app/navigation/nav_engine.h"
#include "../code/app/navigation/nav_adapter.h"
#include "../code/app/navigation/nav_route_record.h"
#include "../code/app/navigation/route_remote.h"
#include "../code/app/vision/vision_pipeline.h"
#include "../code/app/robot_control/robot_control.h"
#include "../code/app/robot_control/small_driver_uart_control.h"
#include "../code/app/remote/remote_comm.h"
#include "../code/app/remote/remote_debug.h"
#include "../code/app/robot_control/jump.h"
#include "../code/app/robot_control/track_elements.h"
#include "../code/control/leg/angle_offset.h"
#include "../code/hmi/indicator/led_buzzer.h"

#ifndef WIFI_SPI_CORE1_TEST
#define WIFI_SPI_CORE1_TEST 0
#endif

#if WIFI_SPI_CORE1_TEST

#define WIFI_CORE0_READY_MAGIC 0x57494649u
#define WIFI_CORE1_STATUS_BOOTED        1u
#define WIFI_CORE1_STATUS_SAW_READY     2u
#define WIFI_CORE1_STATUS_REMOTE_INIT   3u

#pragma location = 0x28006C00
__no_init volatile uint32 g_wifi_core0_ready;
#pragma location = 0x28006C20
__no_init volatile uint32 g_wifi_core1_status;

static void test_debug_line(const char *str)
{
    debug_send_buffer((const uint8 *)str, (uint32)strlen(str));
    debug_send_buffer((const uint8 *)"\r\n", 2u);
}

int main(void)
{
    uint32 camera_frame_count = 0u;
    uint32 last_camera_frame_count = 0u;
    uint32 last_core1_status = 0xFFFFFFFFu;

    g_wifi_core0_ready = 0u;
    g_wifi_core1_status = 0u;
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    test_debug_line("WiFi SPI core1 test boot.");

    test_debug_line("Camera init start.");
    if (mt9v03x_init()) {
        test_debug_line("Camera init failed.");
    } else {
        test_debug_line("Camera init OK.");
    }

    g_wifi_core0_ready = WIFI_CORE0_READY_MAGIC;
    SCB_CleanDCache_by_Addr((uint32 *)&g_wifi_core0_ready, 32u);

    test_debug_line("Core0 ready, CM7_1 is debugger-managed.");

    while(true)
    {
        if (mt9v03x_finish_flag) {
            mt9v03x_finish_flag = 0u;
            camera_frame_count++;
        }

        SCB_InvalidateDCache_by_Addr((uint32 *)&g_wifi_core1_status, 32u);
        if (last_core1_status != g_wifi_core1_status) {
            last_core1_status = g_wifi_core1_status;
            switch (g_wifi_core1_status) {
            case WIFI_CORE1_STATUS_BOOTED:
                test_debug_line("Core1 status: booted.");
                break;
            case WIFI_CORE1_STATUS_SAW_READY:
                test_debug_line("Core1 status: saw core0 ready.");
                break;
            case WIFI_CORE1_STATUS_REMOTE_INIT:
                test_debug_line("Core1 status: remote init.");
                break;
            default:
                test_debug_line("Core1 status: idle.");
                break;
            }
        }

        if (last_camera_frame_count != camera_frame_count) {
            last_camera_frame_count = camera_frame_count;
            test_debug_line("Camera frame updating.");
        } else {
            test_debug_line("Camera no new frame.");
        }

        test_debug_line("Core0 alive.");
        system_delay_ms(1000);
    }
}

#else

Ctrl_Input_t   g_ctrl;
static Nav_Input_t    g_nav_input;
static Vision_Result_t g_vision;

Motor_cmd_duty_t g_motor_cmd;

Sensor_data_t g_sensor_data;

Move_cmd_t g_move_cmd;

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    Cy_SysEnableApplCore(CORE_CM7_1, CY_CORTEX_M7_1_APPL_ADDR);
    zf_log(0, "CM7_1 start requested.");

    remote_debug_init();

    if(!imu_init())
    {
        zf_log(0, "IMU init failed.");
        while(true);
    }

    nav_init(NULL);
    vision_init();

    // UI 运行在 CM7_1，CM7_0 只保留控制/感知/驱动逻辑。
    small_driver_uart_init();
    led_buzzer_init();
    remote_comm_init();
    robot_control_init();

    angle_offset_start(NULL);               // 标定全部四个关节 (左前/左后/右前/右后)

    pit_ms_init(PIT_CH1, 1);

    interrupt_global_enable(0);

    while (!angle_offset_is_done()) {
        imu_update(&g_ctrl);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        led_buzzer_tick(1u);
        system_delay_ms(1);

        if (angle_offset_has_fault()) {
            zf_log(0, "Angle calibration FAILED (timeout). Halting.");
            while (true);
        }
    }
    zf_log(0, "Angle calibration OK.");

    //track_rotate720_start();   // 测试旋转720
    // jump_start(-0.1f);

    while(true)
    {

        imu_update(&g_ctrl);// IMU update: for testing, can be moved to timer ISR
        // g_ctrl.body_pitch / body_roll / gyro_pitch_rate / gyro_yaw_rate (rad, rad/s)

        remote_comm_update(&g_ctrl);

        vision_update(&g_vision);

        nav_input_update_from_ctrl(&g_nav_input, &g_ctrl);
        vision_feed_nav_input(&g_nav_input, &g_vision);
        route_remote_update(&g_nav_input);

        {
            Nav_Route_Record_State_t route_state = nav_route_record_get_state();

            if (route_state.mode == NAV_ROUTE_REPLAYING) {
                Nav_Output_t nav_out = nav_route_replay_update(&g_nav_input);
                nav_apply_ctrl(&g_ctrl, &nav_out);

                /* ── 赛道元素区域响应 ── */
                if (nav_out.region_entered) {
                    switch (nav_out.region) {
                    case NAV_REGION_ROTATE:
                        track_rotate720_start();
                        break;
                    case NAV_REGION_JUMP:
                        jump_start(0.0f);
                        break;
                    case NAV_REGION_SINGLE_BRIDGE:
                    case NAV_REGION_UPHILL:
                        track_bridge_climb_activate();
                        break;
                    case NAV_REGION_SPEED_BUMP:
                        track_bumpy_activate();
                        break;
                    default:
                        break;
                    }
                }
                if (nav_out.region_exited) {
                    switch (nav_out.region) {
                    case NAV_REGION_SINGLE_BRIDGE:
                    case NAV_REGION_UPHILL:
                        track_bridge_climb_deactivate();
                        break;
                    case NAV_REGION_SPEED_BUMP:
                        track_bumpy_deactivate();
                        break;
                    case NAV_REGION_ROTATE:
                        track_rotate720_reset();
                        break;
                    default:
                        break;
                    }
                }
            }
        }

        small_driver_get_angle(&small_driver_value);
        sensor_cmd_update(&g_ctrl, &g_sensor_data, &g_move_cmd);
        led_buzzer_tick(1u);
        system_delay_ms(1);

    }
}

#endif
