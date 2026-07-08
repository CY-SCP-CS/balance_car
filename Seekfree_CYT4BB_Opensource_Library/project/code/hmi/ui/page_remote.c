#include "page_remote.h"

#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "cy_device_headers.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_mt9v03x.h"
#include "../../app/vision/vision_annotate.h"

/* Annotated image buffer — same size as camera frame */
static uint8 g_annotated_image[MT9V03X_H][MT9V03X_W];

static void page_remote_prepare_image(const UI_Frame_t *frame)
{
    SCB_InvalidateDCache_by_Addr(mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);

    if (frame->vision != NULL) {
        vision_annotate(g_annotated_image, mt9v03x_image_temp,
                        frame->vision, frame->vision_mode);
    } else {
        (void)memcpy(g_annotated_image[0], mt9v03x_image_temp[0], MT9V03X_IMAGE_SIZE);
    }
}

void page_remote_init(void)
{
    wifi_spi_init(REMOTE_WIFI_SSID, REMOTE_WIFI_PASSWORD);
    wifi_spi_socket_connect("TCP", REMOTE_TARGET_IP, REMOTE_TARGET_PORT, REMOTE_LOCAL_PORT);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        g_annotated_image,
        MT9V03X_W,
        MT9V03X_H);
}

static void apply_remote_params(void)
{
    seekfree_assistant_data_analysis();

    for (uint8 i = 0u; i < REMOTE_PARAM_CHANNELS; i++) {
        if (!seekfree_assistant_parameter_update_flag[i]) {
            continue;
        }
        seekfree_assistant_parameter_update_flag[i] = 0u;
        remote_debug_send_param(i, seekfree_assistant_parameter[i]);
    }
}

void page_remote_update(const UI_Frame_t *frame)
{
    apply_remote_params();
    page_remote_prepare_image(frame);
    seekfree_assistant_camera_send();

    const Ctrl_Input_t    *fb         = frame->fb;
    const Nav_Output_t    *nav_output = frame->nav_output;
    const Vision_Result_t *vision     = frame->vision;
    const Nav_State_t     *state      = frame->nav_state;

    ui_scope6_send(
        fb != NULL ? fb->body_pitch * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->body_roll * UI_RAD_TO_DEG : 0.0f,
        fb != NULL ? fb->gyro_pitch_rate : 0.0f,
        nav_output != NULL ? nav_output->velocity_cmd : 0.0f,
        vision != NULL ? vision->line_offset : 0.0f,
        ui_segment_or_stop(nav_output, state != NULL ? state->segment_index : 0u));
}
