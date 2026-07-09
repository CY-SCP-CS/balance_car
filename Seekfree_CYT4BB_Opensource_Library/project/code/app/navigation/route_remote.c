#include "route_remote.h"

#include "../remote/remote_comm.h"

/* Route keys: key0 start/finish record, key1 add point,
 * key2 replay recorded route, key3 stop/cancel. */
static bool remote_key_rising(const Remote_State_t *remote, uint8 index)
{
    static uint8 prev_key[4] = {0u, 0u, 0u, 0u};

    if (remote == NULL || index >= 4u) {
        return false;
    }

    uint8 now = remote->key[index] != 0u ? 1u : 0u;
    bool rising = (now != 0u && prev_key[index] == 0u);
    prev_key[index] = now;

    return rising;
}

void route_remote_update(const Nav_Input_t *input)
{
    const Remote_State_t *remote = remote_comm_get_state();

    if (input == NULL || remote == NULL || !remote->connected) {
        return;
    }

    if (remote_key_rising(remote, 3u)) {
        Nav_Route_Record_State_t state = nav_route_record_get_state();

        if (state.mode == NAV_ROUTE_RECORDING) {
            nav_route_record_reset();
        } else {
            nav_route_replay_stop();
        }
        return;
    }

    if (remote_key_rising(remote, 0u)) {
        Nav_Route_Record_State_t state = nav_route_record_get_state();

        if (state.mode == NAV_ROUTE_RECORDING) {
            (void)nav_route_record_keypoint(input);
            (void)nav_route_record_finish();
        } else {
            (void)nav_route_record_start(input);
        }
    }

    if (remote_key_rising(remote, 1u)) {
        (void)nav_route_record_keypoint(input);
    }

    if (remote_key_rising(remote, 2u)) {
        Nav_Route_Record_State_t state = nav_route_record_get_state();

        if (state.mode != NAV_ROUTE_RECORDING) {
            (void)nav_route_replay_start(input);
        }
    }
}
