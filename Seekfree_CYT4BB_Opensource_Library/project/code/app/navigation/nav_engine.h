#ifndef APP_NAVIGATION_NAV_ENGINE_H
#define APP_NAVIGATION_NAV_ENGINE_H

#include "zf_common_typedef.h"

#include "../../common/types.h"

#define NAV_LOOP_DT_S   0.001f
#define NAV_LOOP_DT_MS  1u

typedef enum {
    NAV_ACTION_IDLE = 0,
    NAV_ACTION_GO_STRAIGHT,
    NAV_ACTION_TURN,
    NAV_ACTION_WAIT_LANDMARK,
    NAV_ACTION_STOP
} Nav_Action_t;

typedef enum {
    NAV_LANDMARK_NONE = 0,
    NAV_LANDMARK_CONE,
    NAV_LANDMARK_WHITE_CIRCLE,
    NAV_LANDMARK_STEP
} Nav_Landmark_t;

typedef struct {
    Nav_Action_t action;
    float target_distance_m;
    float target_yaw_deg;
    float target_speed;
    uint32 timeout_ms;
    Nav_Landmark_t landmark;
} Nav_Segment_t;

typedef struct {
    float distance_m;
    float yaw_rad;
    uint32 time_ms;
    Nav_Landmark_t landmark;
    float landmark_offset;
    uint8 landmark_confidence;
    bool obstacle_close;
} Nav_Input_t;

typedef struct {
    float yaw_kp;
    float turn_kp;
    float landmark_kp;
    float steering_limit;
    float distance_tolerance_m;
    float yaw_tolerance_rad;
    uint8 landmark_min_confidence;
} Nav_Config_t;

typedef struct {
    bool active;
    bool finished;
    bool safety_stop;
    uint8 segment_index;
    Nav_Action_t action;
    float segment_distance_m;
    float yaw_error_rad;
} Nav_State_t;

typedef struct {
    float velocity_cmd;
    float steering_cmd;
    bool finished;
    bool safety_stop;
} Nav_Output_t;

void         nav_init                   (const Nav_Config_t *config);
void         nav_set_route              (const Nav_Segment_t *route, uint8 route_len);
void         nav_start                  (const Nav_Input_t *input);
void         nav_stop                   (void);
void         nav_input_update_from_ctrl (Nav_Input_t *input, const Ctrl_Input_t *ctrl);
Nav_Output_t nav_update                 (const Nav_Input_t *input);
Nav_State_t  nav_get_state              (void);
void         nav_apply_ctrl             (Ctrl_Input_t *ctrl, const Nav_Output_t *nav);
Nav_Config_t  nav_get_config            (void);
void          nav_set_config            (const Nav_Config_t *config);
Nav_Config_t *nav_config                (void);

#endif
