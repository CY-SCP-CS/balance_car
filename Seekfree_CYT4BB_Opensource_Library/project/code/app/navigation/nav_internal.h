#ifndef APP_NAVIGATION_NAV_INTERNAL_H
#define APP_NAVIGATION_NAV_INTERNAL_H

#include "nav_engine.h"

#define NAV_PI              3.14159265f
#define NAV_DEG_TO_RAD      (NAV_PI / 180.0f)
#define NAV_RAD_TO_DEG      (180.0f / NAV_PI)

static inline float nav_wrap_pi(float angle)
{
    while (angle > NAV_PI) {
        angle -= 2.0f * NAV_PI;
    }

    while (angle < -NAV_PI) {
        angle += 2.0f * NAV_PI;
    }

    return angle;
}

void nav_route_record_notify_navigation_stopped(bool finished,
                                                bool safety_stop);

#endif
