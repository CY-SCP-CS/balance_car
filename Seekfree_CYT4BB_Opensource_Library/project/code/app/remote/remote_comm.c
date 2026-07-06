#include "remote_comm.h"

#include "remote_protocol.h"
#include "zf_device_lora3a22.h"

#define REMOTE_LPF_ALPHA       0.25f
#define REMOTE_FF_GAIN_BASE    0.25f
#define REMOTE_FF_GAIN_SCALE   0.20f
#define REMOTE_FF_CURVE_POWER  2.0f

static Remote_State_t g_remote_state;
static uint16 g_remote_timeout_ms;
static float g_remote_filtered_joystick[4];
static float g_remote_prev_velocity_cmd;
static float g_remote_prev_steering_cmd;

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
    g_remote_prev_velocity_cmd = 0.0f;
    g_remote_prev_steering_cmd = 0.0f;
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
        g_remote_prev_velocity_cmd = 0.0f;
        g_remote_prev_steering_cmd = 0.0f;
    }

    if (ctrl == NULL) {
        return;
    }

    /* 依据遥控协议注释：
     * joystick[0] = 左摇杆 X（左右）
     * joystick[1] = 左摇杆 Y（前后）
     * 这里把左摇杆前后控制速度，左右控制转向。 */
    float velocity_cmd = -g_remote_state.joystick[1];
    float steering_cmd = -g_remote_state.joystick[0];

    /* 加速度前馈：让大幅度推杆时前馈更强，接近极限时更有冲劲。 */
    float velocity_mag = remote_absf(velocity_cmd);
    float steering_mag = remote_absf(steering_cmd);

    float velocity_curve = powf(velocity_mag, REMOTE_FF_CURVE_POWER);
    float steering_curve = powf(steering_mag, REMOTE_FF_CURVE_POWER);

    float velocity_gain = REMOTE_FF_GAIN_BASE * (1.0f + REMOTE_FF_GAIN_SCALE * velocity_curve);
    float steering_gain = REMOTE_FF_GAIN_BASE * (1.0f + REMOTE_FF_GAIN_SCALE * steering_curve);

    float velocity_ff = (velocity_cmd - g_remote_prev_velocity_cmd) * velocity_gain;
    float steering_ff = (steering_cmd - g_remote_prev_steering_cmd) * steering_gain;

    ctrl->velocity_cmd = remote_clamp(velocity_cmd + velocity_ff, -1.0f, 1.0f);
    ctrl->steering_cmd = remote_clamp(steering_cmd + steering_ff, -1.0f, 1.0f);

    g_remote_prev_velocity_cmd = velocity_cmd;
    g_remote_prev_steering_cmd = steering_cmd;
}

const Remote_State_t *remote_comm_get_state(void)
{
    return &g_remote_state;
}
