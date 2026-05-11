#ifndef APP_VISION_VISION_PIPELINE_H
#define APP_VISION_VISION_PIPELINE_H

#include "../../app/navigation/nav_engine.h"

typedef struct {
    float          line_offset;         /* normalised track-centre offset, -1 to +1 */
    uint8          line_confidence;     /* 0-10 */
    Nav_Landmark_t landmark;
    float          landmark_offset;
    uint8          landmark_confidence;
    bool           obstacle_close;
} Vision_Result_t;

void vision_init(void);
bool vision_update(Vision_Result_t *result);
void vision_feed_nav_input(Nav_Input_t *input, const Vision_Result_t *v);

#endif
