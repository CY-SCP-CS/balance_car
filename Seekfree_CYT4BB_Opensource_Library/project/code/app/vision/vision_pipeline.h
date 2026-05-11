#ifndef APP_VISION_VISION_PIPELINE_H
#define APP_VISION_VISION_PIPELINE_H

#include <stdbool.h>
#include "../../app/navigation/nav_engine.h"

typedef struct {
    float          line_offset;          /* normalised track-centre offset, -1 to +1 */
    uint8_t        line_confidence;      /* 0–10 */
    Nav_Landmark_t landmark;
    float          landmark_offset;
    uint8_t        landmark_confidence;
    bool           obstacle_close;
} Vision_Result_t;

void vision_init(void);

/* Call once per control loop. Returns true when a fresh camera frame was
   processed, false if the previous result is still valid. */
bool vision_update(Vision_Result_t *result);

/* Populate the fields in *input that come from vision. */
void vision_feed_nav_input(Nav_Input_t *input, const Vision_Result_t *v);

#endif
