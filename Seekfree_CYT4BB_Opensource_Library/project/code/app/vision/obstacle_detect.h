#ifndef APP_VISION_OBSTACLE_DETECT_H
#define APP_VISION_OBSTACLE_DETECT_H

#include "../../../../libraries/zf_device/zf_device_mt9v03x.h"
#include "../../app/navigation/nav_engine.h"

typedef struct {
    bool           obstacle_close;
    Nav_Landmark_t landmark;
    float          landmark_offset;  /* normalised -1.0 to +1.0 */
    uint8          landmark_confidence;
} Obstacle_Result_t;

void obstacle_detect(const uint8 image[MT9V03X_H][MT9V03X_W], Obstacle_Result_t *result);

#endif
