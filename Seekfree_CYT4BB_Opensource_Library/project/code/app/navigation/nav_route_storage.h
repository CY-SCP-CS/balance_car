#ifndef APP_NAVIGATION_NAV_ROUTE_STORAGE_H
#define APP_NAVIGATION_NAV_ROUTE_STORAGE_H

#include "nav_route_record.h"

void nav_route_storage_init(void);
bool nav_route_storage_save(const Nav_Segment_t *route, uint8 route_len);
bool nav_route_storage_load_history(Nav_Segment_t *route,
                                    uint8 max_route_len,
                                    uint8 *route_len,
                                    uint8 history_index);
bool nav_route_storage_load(Nav_Segment_t *route,
                            uint8 max_route_len,
                            uint8 *route_len);
bool nav_route_storage_load_previous(Nav_Segment_t *route,
                                     uint8 max_route_len,
                                     uint8 *route_len);

#endif
