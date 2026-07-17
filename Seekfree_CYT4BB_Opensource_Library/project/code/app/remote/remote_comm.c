#include "remote_comm.h"

#include "remote_protocol.h"
#include "../../hmi/indicator/led_buzzer.h"
#include "zf_device_lora3a22.h"

#define REMOTE_LPF_ALPHA       0.25f
#define REMOTE_BEEP_KEY        2u
#define REMOTE_CMD_DT_S        0.001f
#define REMOTE_SPEED_ACCEL     3.0f
#define REMOTE_SPEED_DECEL     5.0f
#define REMOTE_SPEED_STOP_EPS  0.002f

static Remote_State_t g_remote_state;
static uint16 g_remote_timeout_ms;
static float g_remote_filtered_joystick[4];
static float g_remote_velocity_target;
static uint8 g_remote_beep_key_prev;

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

static float remote_approach(float current, float target, float max_delta)
{
    float delta = target - current;

    if (delta > max_delta) {
        delta = max_delta;
    } else if (delta < -max_delta) {
        delta = -max_delta;
    }

    return current + delta;
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
    return frame->joystick[channel];
}

static bool remote_read_lora_frame(lora3a22_uart_transfer_dat_struct *frame)
{
    if (frame == NULL || !lora3a22_finsh_flag) {
        return false;
    }

    *frame = lora3a22_uart_transfer;
    lora3a22_finsh_flag = 0u;

    return true;
}

static void remote_clear_runtime_state(void)
{
    for (uint8 i = 0u; i < 4u; i++) {
        g_remote_state.joystick[i] = 0;
        g_remote_filtered_joystick[i] = 0.0f;
        g_remote_state.key[i] = 0u;
        g_remote_state.switch_key[i] = 0u;
    }
    g_remote_velocity_target = 0.0f;
    g_remote_beep_key_prev = 0u;
}

static void remote_update_key_buzzer(void)
{
    uint8 key_now = g_remote_state.key[REMOTE_BEEP_KEY] != 0u ? 1u : 0u;

    if (key_now != 0u && g_remote_beep_key_prev == 0u) {
        buzzer_beep(BEEP_SHORT);
    }

    g_remote_beep_key_prev = key_now;
}

void remote_comm_init(void)
{
    memset(&g_remote_state, 0, sizeof(g_remote_state));
    memset(g_remote_filtered_joystick, 0, sizeof(g_remote_filtered_joystick));
    g_remote_velocity_target = 0.0f;
    g_remote_beep_key_prev = 0u;
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
        remote_update_key_buzzer();
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
        g_remote_velocity_target = 0.0f;
        ctrl->velocity_cmd = 0.0f;
        ctrl->steering_cmd = 0.0f;
        ctrl->yaw_target_valid = false;
        ctrl->yaw_target_rad = 0.0f;
        ctrl->on_bridge = false;
        return;
    }

    ctrl->yaw_target_valid = false;
    ctrl->yaw_target_rad = 0.0f;
    ctrl->on_bridge = (g_remote_state.switch_key[0] != 0u);

    /* 依据遥控协议注释：
     * joystick[2] = 右摇杆 X（左右）
     * joystick[1] = 左摇杆 Y（前后）
     * 这里把左摇杆前后控制速度，左右控制转向。 */
    float velocity_cmd = g_remote_filtered_joystick[1];
    float steering_cmd = -g_remote_filtered_joystick[2];
    float speed_rate = REMOTE_SPEED_ACCEL;

    if (velocity_cmd * g_remote_velocity_target < 0.0f ||
        remote_absf(velocity_cmd) < remote_absf(g_remote_velocity_target)) {
        speed_rate = REMOTE_SPEED_DECEL;
    }

    g_remote_velocity_target = remote_approach(g_remote_velocity_target,
        velocity_cmd, speed_rate * REMOTE_CMD_DT_S);
    if (velocity_cmd == 0.0f &&
        remote_absf(g_remote_velocity_target) < REMOTE_SPEED_STOP_EPS) {
        g_remote_velocity_target = 0.0f;
    }

    ctrl->velocity_cmd = remote_clamp(g_remote_velocity_target, -1.0f, 1.0f);
    ctrl->steering_cmd = remote_clamp(steering_cmd, -1.0f, 1.0f);
}

const Remote_State_t *remote_comm_get_state(void)
{
    return &g_remote_state;
}
