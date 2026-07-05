#include "remote_comm.h"

#include "remote_protocol.h"
#include "zf_device_lora3a22.h"

#define REMOTE_LPF_ALPHA 0.25f

static Remote_State_t g_remote_state;
static uint16 g_remote_timeout_ms;
static float g_remote_filtered_joystick[4];

static float remote_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static float remote_clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static float remote_normalize_joystick(int16 raw)
{
    float value = (float)raw / REMOTE_LORA_JOYSTICK_MAX;

    value = remote_clamp(value, -1.0f, 1.0f);
    if (remote_absf(value) < REMOTE_LORA_JOYSTICK_DEADBAND) {
        value = 0.0f;
    }

    return value;
}

static int16 remote_get_raw_joystick(uint8 channel)
{
    uint8 offset = (uint8)(2u + channel * 2u);

    return (int16)((uint16)lora3a22_uart_data[offset] << 8 |
                   (uint16)lora3a22_uart_data[offset + 1u]);
}

void remote_comm_init(void)
{
    memset(&g_remote_state, 0, sizeof(g_remote_state));
    memset(g_remote_filtered_joystick, 0, sizeof(g_remote_filtered_joystick));
    g_remote_timeout_ms = REMOTE_LORA_TIMEOUT_MS;
    lora3a22_init();
}

void remote_comm_update(Ctrl_Input_t *ctrl)
{
    g_remote_state.frame_updated = false;

    if (lora3a22_finsh_flag) {
        lora3a22_finsh_flag = 0;

        for (uint8 i = 0u; i < 4u; i++) {
            float raw_value = remote_normalize_joystick(remote_get_raw_joystick(i));
            g_remote_filtered_joystick[i] += (raw_value - g_remote_filtered_joystick[i]) * REMOTE_LPF_ALPHA;
            g_remote_state.joystick[i] = g_remote_filtered_joystick[i];
            g_remote_state.key[i] = lora3a22_uart_transfer.key[i];
            g_remote_state.switch_key[i] = lora3a22_uart_transfer.switch_key[i];
        }

        g_remote_state.connected = true;
        g_remote_state.frame_updated = true;
        g_remote_timeout_ms = 0u;
    } else if (g_remote_timeout_ms < REMOTE_LORA_TIMEOUT_MS) {
        g_remote_timeout_ms++;
    } else {
        g_remote_state.connected = false;
        for (uint8 i = 0u; i < 4u; i++) {
            g_remote_state.joystick[i] = 0.0f;
            g_remote_filtered_joystick[i] = 0.0f;
        }
    }

    if (ctrl == NULL) {
        return;
    }

    /* 依据遥控协议注释：
     * joystick[0] = 左摇杆 X（左右）
     * joystick[1] = 左摇杆 Y（前后）
     * 这里把左摇杆前后控制速度，左右控制转向。 */
    ctrl->velocity_cmd = -g_remote_state.joystick[1];
    ctrl->steering_cmd = -g_remote_state.joystick[0];
}

const Remote_State_t *remote_comm_get_state(void)
{
    return &g_remote_state;
}
