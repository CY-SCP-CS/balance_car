#include "route_remote.h"

#include "../../hmi/indicator/led_buzzer.h"
#include "../remote/remote_comm.h"

/* key[2]: add record point; switch_key[2]: replay; switch_key[3]: record mode. */
#define ROUTE_RECORD_KEY        2u
#define ROUTE_PLAY_SWITCH       2u
#define ROUTE_RECORD_SWITCH     3u

static uint8 g_prev_key[4] = {0u, 0u, 0u, 0u};
static uint8 g_prev_record_switch;
static uint8 g_prev_play_switch;
static bool g_prev_connected;

static bool remote_key_rising(const Remote_State_t *remote, uint8 index)
{
    if (remote == NULL || index >= 4u) {
        return false;
    }

    uint8 now = remote->key[index] != 0u ? 1u : 0u;
    bool rising = (now != 0u && g_prev_key[index] == 0u);
    g_prev_key[index] = now;

    return rising;
}

static void remote_key_sync(const Remote_State_t *remote, uint8 index)
{
    if (remote == NULL || index >= 4u) {
        return;
    }

    g_prev_key[index] = remote->key[index] != 0u ? 1u : 0u;
}

void route_remote_update(const Nav_Input_t *input)
{
    const Remote_State_t *remote = remote_comm_get_state();
    uint8 record_switch;
    uint8 play_switch;
    bool record_switch_rising;
    bool record_switch_falling;
    bool play_switch_rising;
    Nav_Route_Record_State_t state;

    if (remote == NULL) {
        return;
    }

    record_switch = remote->switch_key[ROUTE_RECORD_SWITCH] != 0u ? 1u : 0u;
    play_switch = remote->switch_key[ROUTE_PLAY_SWITCH] != 0u ? 1u : 0u;

    if (!remote->connected) {
        g_prev_connected = false;
        state = nav_route_record_get_state();
        if (state.mode == NAV_ROUTE_REPLAYING) {
            nav_route_replay_stop();
        }
        return;
    }

    if (input == NULL) {
        return;
    }

    if (!g_prev_connected) {
        g_prev_connected = true;
        g_prev_record_switch = record_switch;
        g_prev_play_switch = play_switch;
        remote_key_sync(remote, ROUTE_RECORD_KEY);
        return;
    }

    record_switch_rising = (record_switch != 0u && g_prev_record_switch == 0u);
    record_switch_falling = (record_switch == 0u && g_prev_record_switch != 0u);
    play_switch_rising = (play_switch != 0u && g_prev_play_switch == 0u);
    g_prev_record_switch = record_switch;
    g_prev_play_switch = play_switch;

    if (record_switch_rising) {
        nav_route_replay_stop();
        if (nav_route_record_start(input)) {
            buzzer_beep(BEEP_TRIPLE);
        }
        remote_key_sync(remote, ROUTE_RECORD_KEY);
    }

    if (record_switch != 0u) {
        state = nav_route_record_get_state();
        if (state.mode != NAV_ROUTE_RECORDING) {
            if (nav_route_record_start(input)) {
                buzzer_beep(BEEP_TRIPLE);
            }
            remote_key_sync(remote, ROUTE_RECORD_KEY);
        }

        if (state.mode == NAV_ROUTE_RECORDING) {
            if (remote_key_rising(remote, ROUTE_RECORD_KEY)) {
                (void)nav_route_record_keypoint(input);
            }
        }
        return;
    }

    if (record_switch_falling) {
        state = nav_route_record_get_state();
        if (state.mode == NAV_ROUTE_RECORDING) {
            (void)nav_route_record_keypoint(input);
            if (nav_route_record_finish()) {
                buzzer_beep(BEEP_LONG);
            }
        }
    }

    state = nav_route_record_get_state();
    if (play_switch == 0u && state.mode == NAV_ROUTE_REPLAYING) {
        nav_route_replay_stop();
        return;
    }

    if (play_switch_rising && state.mode != NAV_ROUTE_RECORDING) {
        if (nav_route_replay_start(input)) {
            buzzer_beep(BEEP_DOUBLE_LONG);
        }
    }
}
