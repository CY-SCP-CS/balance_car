#ifndef CONFIG_VISION_DEFAULTS_H
#define CONFIG_VISION_DEFAULTS_H

/* ---- Line detection ---- */
#define VISION_LINE_ROW_START       80u   /* first row scanned (from top, 0-based) */
#define VISION_LINE_ROW_END         115u  /* last row scanned  */
#define VISION_LINE_MIN_WIDTH_PX    20u   /* min plausible track width in pixels   */
#define VISION_LINE_MAX_WIDTH_PX    170u  /* max plausible track width in pixels   */
#define VISION_LINE_THRESH_LOW      80u   /* lower bound for adaptive row threshold */
#define VISION_LINE_THRESH_HIGH     200u  /* upper bound for adaptive row threshold */

/* ---- Obstacle proximity ---- */
#define VISION_OBS_ROW_START        96u   /* bottom rows checked for close obstacle */
#define VISION_OBS_ROW_END          115u
#define VISION_OBS_COL_START        64u   /* centre column band */
#define VISION_OBS_COL_END          124u
#define VISION_OBS_DARK_THRESH      80u   /* pixel is "dark" if below this */
#define VISION_OBS_DARK_RATIO_NUM   6u    /* obstacle_close when dark/total > 6/10 */
#define VISION_OBS_DARK_RATIO_DEN   10u

/* ---- White circle landmark ---- */
#define VISION_CIRCLE_ROW_START     50u
#define VISION_CIRCLE_ROW_END       80u
#define VISION_CIRCLE_COL_START     44u
#define VISION_CIRCLE_COL_END       144u
#define VISION_CIRCLE_BRIGHT_THRESH 200u  /* pixel counts as "white" above this */
#define VISION_CIRCLE_MIN_PIXELS    300u  /* minimum white pixels to confirm circle */

/* ---- Cone landmark ---- */
#define VISION_CONE_ROW_START       20u
#define VISION_CONE_ROW_END         60u
#define VISION_CONE_COL_START       54u
#define VISION_CONE_COL_END         134u
#define VISION_CONE_LO_THRESH       90u   /* cone pixel range in grayscale */
#define VISION_CONE_HI_THRESH       190u
#define VISION_CONE_MIN_PIXELS      200u

/* ---- Step landmark ---- */
#define VISION_STEP_ROW_A           55u   /* rows compared for horizontal edge */
#define VISION_STEP_ROW_B           65u
#define VISION_STEP_EDGE_THRESH     40u   /* mean brightness difference to confirm step */

/* ---- Minefield (white border, blue/dark background) ---- */
#define VISION_MF_WHITE_THRESH      160u  /* pixel counts as white border above this */
#define VISION_MF_MIN_WHITE_PX      10u   /* min white pixels per col/row to count as edge */
#define VISION_MF_DANGER_MARGIN     20u   /* pixels from frame edge → trigger near-warning */

/* ---- Stair (blue middle step, grayscale intensity band) ---- */
#define VISION_STAIR_LO_THRESH      80u   /* lower grayscale bound for blue step; calibrate on site */
#define VISION_STAIR_HI_THRESH      160u  /* upper grayscale bound */
#define VISION_STAIR_MIN_COLS       40u   /* min pixels per row to count as stair row */
#define VISION_STAIR_ROI_TOP        20u
#define VISION_STAIR_ROI_BOT        100u
#define VISION_STAIR_HEIGHT_CM      10.0f /* physical height of one step; measure before race */
#define VISION_STAIR_FOCAL_CONST    600.0f/* calibrated: HEIGHT_CM * f_px; fill after calibration */

/* ---- Bridge (black wedges on white PVC) ---- */
#define VISION_BRIDGE_DARK_THRESH   60u   /* pixel counts as black wedge below this */
#define VISION_BRIDGE_WEDGE_MIN_W   4u    /* min wedge width in pixels */
#define VISION_BRIDGE_WEDGE_MAX_W   30u   /* max wedge width in pixels */
#define VISION_BRIDGE_ROI_TOP       60u   /* ROI covering bridge surface (lower half of frame) */
#define VISION_BRIDGE_ROI_BOT       100u
#define VISION_BRIDGE_ALIGN_THRESH  0.08f /* |lateral_offset| < this → aligned */

#endif
