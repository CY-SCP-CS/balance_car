#ifndef APP_VISION_VISION_PIPELINE_H
#define APP_VISION_VISION_PIPELINE_H

#include "../../app/navigation/nav_engine.h"
#include "minefield_detect.h"
#include "stair_detect.h"
#include "bridge_detect.h"

typedef enum {
    VISION_MODE_LINE      = 0,  /* default: line tracking only */
    VISION_MODE_MINEFIELD = 1,  /* 科目2: white border detection */
    VISION_MODE_BRIDGE    = 2,  /* 科目3: black wedge counting */
    VISION_MODE_STAIR     = 3,  /* 科目3: blue step edge ranging */
    VISION_MODE_CONE      = 4,  /* 科目1: cone assist (uses obstacle_detect) */
} Vision_Mode_t;

typedef struct {
    float              line_offset;         /* normalised track-centre offset, -1 to +1 */
    uint8              line_confidence;     /* 0-10 */
    Nav_Landmark_t     landmark;
    bool               obstacle_close;
    Minefield_Result_t minefield;
    Stair_Result_t     stair;
    Bridge_Result_t    bridge;
} Vision_Result_t;

void vision_init(void);
bool vision_update(Vision_Result_t *result);
void vision_set_mode(Vision_Mode_t mode);
Vision_Mode_t vision_get_mode(void);
void vision_feed_nav_input(Nav_Input_t *input, const Vision_Result_t *v);

#endif
