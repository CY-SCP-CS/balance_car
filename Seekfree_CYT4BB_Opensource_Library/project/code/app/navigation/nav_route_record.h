#ifndef APP_NAVIGATION_NAV_ROUTE_RECORD_H
#define APP_NAVIGATION_NAV_ROUTE_RECORD_H

#include "nav_engine.h"

#define NAV_RECORD_MAX_KEYPOINTS  64u /* 最多保存的关键点数量；调大会增加路线容量和flash占用 */
#define NAV_RECORD_MAX_SEGMENTS   64u /* 最多支持的路线段数量；应与关键点容量保持匹配 */

typedef enum {
    NAV_ROUTE_IDLE = 0,
    NAV_ROUTE_RECORDING,
    NAV_ROUTE_READY,
    NAV_ROUTE_REPLAYING,
} Nav_Route_Mode_t;

typedef struct {
    /* Route-local coordinates: origin = record start, +x = start yaw. */
    float x_m;
    float y_m;
    float distance_m;
    float yaw_rad;
    uint32 time_ms;
    Nav_Route_Point_Action_t action;
} Nav_Keypoint_t;

typedef struct {
    Nav_Route_Mode_t mode;
    uint8 keypoint_count;
    uint8 replay_index;
    bool route_ready;
    bool overflow;
} Nav_Route_Record_State_t;

bool nav_route_record_start(const Nav_Input_t *input);
bool nav_route_record_keypoint(const Nav_Input_t *input);
bool nav_route_record_keypoint_with_action(const Nav_Input_t *input,
                                           Nav_Route_Point_Action_t action);
bool nav_route_record_finish(void);
bool nav_route_record_save_reserved(void);
bool nav_route_record_save_reserved_slot(uint8 reserved_slot);
void nav_route_record_reset(void);
bool nav_route_record_load_saved_history(uint8 history_index);
bool nav_route_record_load_saved(void);
bool nav_route_record_load_previous_saved(void);
bool nav_route_record_load_reserved_slot(uint8 reserved_slot);
bool nav_route_replay_start(const Nav_Input_t *input);
Nav_Output_t nav_route_replay_update(const Nav_Input_t *input);
bool nav_route_replay_anchor_to_next_action(const Nav_Input_t *input,
                                            Nav_Route_Point_Action_t action);
void nav_route_replay_stop(void);
Nav_Route_Record_State_t nav_route_record_get_state(void);

#endif
