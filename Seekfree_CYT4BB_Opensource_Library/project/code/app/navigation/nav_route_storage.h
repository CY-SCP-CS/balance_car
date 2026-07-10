#ifndef APP_NAVIGATION_NAV_ROUTE_STORAGE_H
#define APP_NAVIGATION_NAV_ROUTE_STORAGE_H

#include "nav_route_record.h"

void nav_route_storage_init(void);
bool nav_route_storage_save(const Nav_Keypoint_t *keypoints,
                            uint8 keypoint_count);
bool nav_route_storage_load_history(Nav_Keypoint_t *keypoints,
                                    uint8 max_keypoints,
                                    uint8 *keypoint_count,
                                    uint8 history_index);
bool nav_route_storage_load(Nav_Keypoint_t *keypoints,
                            uint8 max_keypoints,
                            uint8 *keypoint_count);
bool nav_route_storage_load_previous(Nav_Keypoint_t *keypoints,
                                     uint8 max_keypoints,
                                     uint8 *keypoint_count);

#endif
