#include "remote_comm.h"

#include "remote_protocol.h"
#include "zf_common_interrupt.h"
#include "zf_device_lora3a22.h"

#define REMOTE_LPF_ALPHA       0.25f

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

static int16 remote_get_raw_joystick(const lora3a22_uart_transfer_dat_struct *frame, uint8 channel)
{
    const uint8 *frame_data = (const uint8 *)frame;
    uint8 offset = (uint8)(2u + channel * 2u);

    return (int16)((uint16)frame_data[offset] << 8 |
                   (uint16)frame_data[offset + 1u]);
}

static bool remote_read_lora_frame(lora3a22_uart_transfer_dat_struct *frame)
{
    uint32 interrupt_state;
    bool has_frame = false;

    if (frame == NULL) {
        return false;
    }

    interrupt_state = interrupt_global_disable();
    if (lora3a22_finsh_flag) {
        *frame = lora3a22_uart_transfer;
        lora3a22_finsh_flag = 0u;
        has_frame = true;
    }
    interrupt_global_enable(interrupt_state);

    return has_frame;
}

static void remote_clear_runtime_state(void)
{
    for (uint8 i = 0u; i < 4u; i++) {
        g_remote_state.joystick[i] = 0;
        g_remote_filtered_joystick[i] = 0.0f;
        g_remote_state.key[i] = 0u;
        g_remote_state.switch_key[i] = 0u;
    }
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
    lora3a22_uart_transfer_dat_struct frame;

    g_remote_state.frame_updated = false;

    if (remote_read_lora_frame(&frame)) {
        for (uint8 i = 0u; i < 4u; i++) {
            int16 raw_value = remote_get_raw_joystick(&frame, i);
            float normalized_value = remote_normalize_joystick(raw_value);

            g_remote_filtered_joystick[i] +=
                (normalized_value - g_remote_filtered_joystick[i]) * REMOTE_LPF_ALPHA;
            if (remote_absf(g_remote_filtered_joystick[i]) < REMOTE_LORA_JOYSTICK_DEADBAND) {
                g_remote_filtered_joystick[i] = 0.0f;
            }
            g_remote_state.joystick[i] = raw_value;
            g_remote_state.key[i] = frame.key[i];
            g_remote_state.switch_key[i] = frame.switch_key[i];
        }

        g_remote_state.connected = true;
        g_remote_state.frame_updated = true;
        g_remote_timeout_ms = 0u;
    } else if (g_remote_timeout_ms < REMOTE_LORA_TIMEOUT_MS) {
        g_remote_timeout_ms++;
    } else {
        g_remote_state.connected = false;
        remote_clear_runtime_state();
    }

    if (ctrl == NULL) {
        return;
    }

    if (!g_remote_state.connected) {
        ctrl->velocity_cmd = 0.0f;
        ctrl->steering_cmd = 0.0f;
        ctrl->on_bridge = false;
        return;
    }

    ctrl->on_bridge = (g_remote_state.switch_key[0] != 0u);

    /* 依据遥控协议注释：
     * joystick[2] = 右摇杆 X（左右）
     * joystick[1] = 左摇杆 Y（前后）
     * 这里把左摇杆前后控制速度，左右控制转向。 */
    float velocity_cmd = g_remote_filtered_joystick[1];
    float steering_cmd = g_remote_filtered_joystick[2];

    ctrl->velocity_cmd = remote_clamp(velocity_cmd, -1.0f, 1.0f);
    ctrl->steering_cmd = remote_clamp(steering_cmd, -1.0f, 1.0f);
}

const Remote_State_t *remote_comm_get_state(void)
{
    return &g_remote_state;
}
