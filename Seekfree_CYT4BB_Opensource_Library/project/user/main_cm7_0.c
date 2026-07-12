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

#define WIFI_SPI_STANDALONE_TEST 1

#define WIFI_TEST_SSID          "BNGU"
#define WIFI_TEST_PASSWORD      "12345678"
#define WIFI_TEST_TARGET_IP     "192.168.1.102"
#define WIFI_TEST_TARGET_PORT   "8086"
#define WIFI_TEST_LOCAL_PORT    "6666"

#if WIFI_SPI_STANDALONE_TEST

static void test_debug_line(const char *str)
{
    debug_send_buffer((const uint8 *)str, (uint32)strlen(str));
    debug_send_buffer((const uint8 *)"\r\n", 2u);
}

static void test_debug_label_value(const char *label, const char *value)
{
    debug_send_buffer((const uint8 *)label, (uint32)strlen(label));
    debug_send_buffer((const uint8 *)value, (uint32)strlen(value));
    debug_send_buffer((const uint8 *)"\r\n", 2u);
}

int main(void)
{
    uint8 state;

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    test_debug_line("WiFi SPI standalone test boot.");
    test_debug_line("WiFi SPI init start.");

    state = wifi_spi_init(WIFI_TEST_SSID, WIFI_TEST_PASSWORD);
    if(state)
    {
        test_debug_line("WiFi SPI init failed.");
        while(true)
        {
            system_delay_ms(1000);
        }
    }

    test_debug_line("WiFi SPI init OK.");
    test_debug_label_value("version: ", wifi_spi_version);
    test_debug_label_value("mac: ", wifi_spi_mac_addr);
    test_debug_label_value("ip: ", wifi_spi_ip_addr_port);

    test_debug_line("WiFi TCP connect start.");
    state = wifi_spi_socket_connect("TCP", WIFI_TEST_TARGET_IP, WIFI_TEST_TARGET_PORT, WIFI_TEST_LOCAL_PORT);
    if(state)
    {
        test_debug_line("WiFi TCP connect failed.");
    }
    else
    {
        test_debug_line("WiFi TCP connect OK.");
        test_debug_label_value("ip: ", wifi_spi_ip_addr_port);
    }

    while(true)
    {
        test_debug_line("WiFi SPI test alive.");
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
