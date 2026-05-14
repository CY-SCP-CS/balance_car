#include "page_remote.h"

#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_mt9v03x.h"

#define RAD_TO_DEG  (180.0f / 3.14159265f)

static float *g_bindings[REMOTE_PARAM_CHANNELS];

void remote_page_init(void)
{
    wifi_spi_init(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD);
    wifi_spi_socket_connect("TCP", REMOTE_TARGET_IP, REMOTE_TARGET_PORT, REMOTE_LOCAL_PORT);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        mt9v03x_image,
        MT9V03X_W,
        MT9V03X_H);
}

void remote_param_bind(uint8 channel, float *target)
{
    if (channel < REMOTE_PARAM_CHANNELS) {
        g_bindings[channel] = target;
    }
}

static void apply_remote_params(void)
{
    seekfree_assistant_data_analysis();

    for (uint8 i = 0u; i < REMOTE_PARAM_CHANNELS; i++) {
        if (!seekfree_assistant_parameter_update_flag[i] || g_bindings[i] == NULL) {
            continue;
        }
        seekfree_assistant_parameter_update_flag[i] = 0u;
        *g_bindings[i] = seekfree_assistant_parameter[i];
    }
}

void remote_page_update(const Ctrl_Input_t    *fb,
                        const Nav_Input_t     *nav_input,
                        const Nav_Output_t    *nav_output,
                        const Vision_Result_t *vision)
{
    apply_remote_params();

    seekfree_assistant_camera_send();

    Nav_State_t state = nav_get_state();

    seekfree_assistant_oscilloscope_data.channel_num = 6;
    seekfree_assistant_oscilloscope_data.data[0] =
        fb != NULL ? fb->body_pitch * RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[1] =
        fb != NULL ? fb->body_roll * RAD_TO_DEG : 0.0f;
    seekfree_assistant_oscilloscope_data.data[2] =
        nav_output != NULL ? nav_output->steering_cmd : 0.0f;
    seekfree_assistant_oscilloscope_data.data[3] =
        nav_output != NULL ? nav_output->velocity_cmd : 0.0f;
    seekfree_assistant_oscilloscope_data.data[4] =
        vision != NULL ? vision->line_offset : 0.0f;
    seekfree_assistant_oscilloscope_data.data[5] =
        (nav_output != NULL && nav_output->safety_stop) ? -1.0f : (float)state.segment_index;

    (void)nav_input;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
