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
#define VISION_CIRCLE_CONF_SCALE    600u  /* pixel count mapped to confidence 10 */

/* ---- Cone landmark ---- */
#define VISION_CONE_ROW_START       20u
#define VISION_CONE_ROW_END         60u
#define VISION_CONE_COL_START       54u
#define VISION_CONE_COL_END         134u
#define VISION_CONE_LO_THRESH       90u   /* cone pixel range in grayscale */
#define VISION_CONE_HI_THRESH       190u
#define VISION_CONE_MIN_PIXELS      200u
#define VISION_CONE_CONF_SCALE      500u

/* ---- Step landmark ---- */
#define VISION_STEP_ROW_A           55u   /* rows compared for horizontal edge */
#define VISION_STEP_ROW_B           65u
#define VISION_STEP_EDGE_THRESH     40u   /* mean brightness difference to confirm step */

#endif
