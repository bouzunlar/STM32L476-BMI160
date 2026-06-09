#ifndef DETECTION_INC_DETECTION_H_
#define DETECTION_INC_DETECTION_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "fusion.h"

/* Sliding-window length: 64 samples @ 100 Hz = 640 ms
 * Longer than 32 for more stable arc/variance statistics. */
#define DETECTION_WINDOW_SIZE                 64u

#define DET_TOLERANCE_STRICT                  0
#define DET_TOLERANCE_NORMAL                  1
#define DET_TOLERANCE_LOOSE                   2

#ifndef DET_TOLERANCE_PROFILE
#define DET_TOLERANCE_PROFILE                 DET_TOLERANCE_NORMAL
#endif

#if (DET_TOLERANCE_PROFILE == DET_TOLERANCE_STRICT)
#define DET_YAW_RATE_MEAN_DPS_MIN             (200.0f)
#define DET_ALIN_VAR_G2_MIN                   (0.020f)
#define DET_COV_K_FACTOR                      (0.50f)
#define DET_VAR_RATIO_MAX                     (2.0f)
#define DET_MIN_ZERO_CROSS_EACH_AXIS          (2u)
#define DET_YAW_QUARTERS_REQUIRED             (4u)
#define DET_MIN_ARC_DEG                       (100.0f)
#elif (DET_TOLERANCE_PROFILE == DET_TOLERANCE_LOOSE)
#define DET_YAW_RATE_MEAN_DPS_MIN             (60.0f)
#define DET_ALIN_VAR_G2_MIN                   (0.008f)
#define DET_COV_K_FACTOR                      (0.85f)
#define DET_VAR_RATIO_MAX                     (6.0f)
#define DET_MIN_ZERO_CROSS_EACH_AXIS          (0u)
#define DET_YAW_QUARTERS_REQUIRED             (0u)
#define DET_MIN_ARC_DEG                       (45.0f)
#else
#define DET_YAW_RATE_MEAN_DPS_MIN             (150.0f)
#define DET_ALIN_VAR_G2_MIN                   (0.011f)
#define DET_COV_K_FACTOR                      (0.65f)
#define DET_VAR_RATIO_MAX                     (3.0f)
#define DET_MIN_ZERO_CROSS_EACH_AXIS          (2u)
#define DET_YAW_QUARTERS_REQUIRED             (4u)
#define DET_MIN_ARC_DEG                       (130.0f)
#endif

/* Full-revolution threshold for the cumulative accel-vector arc (a half
 * circle tops out ~256 deg, so 360 cleanly separates them). */
#ifndef DET_CUM_ARC_THRESHOLD_DEG
#define DET_CUM_ARC_THRESHOLD_DEG             (360.0f)
#endif

/* Min |aXY| (g) for the accel-vector angle to be trusted (else it's noise). */
#ifndef DET_ALIN_ANGLE_MIN_G
#define DET_ALIN_ANGLE_MIN_G                  (0.05f)
#endif

/* Upper XY-variance bound; real circles peak ~0.37 g², faster/violent
 * half-arcs sit above 0.40 and are rejected. */
#ifndef DET_ALIN_VAR_G2_MAX
#define DET_ALIN_VAR_G2_MAX                   (0.40f)
#endif

/* Min mean |yawRate| (dps) for the wrist to count as rotating*/
#ifndef DET_YAW_ABS_MEAN_MIN_DPS
#define DET_YAW_ABS_MEAN_MIN_DPS              (50.0f)
#endif

/* --- Accel-vector arc integration constants --- */

/* Angle math for the centripetal accel-vector arc. */
#define DET_PI_RAD                            (3.14159265f)
#define DET_TWO_PI_RAD                        (6.28318531f)
#define DET_RAD_TO_DEG                        (57.29578f)

/* Accel-vector EMA smoothing factor (0..1); higher = smoother, more lag. */
#define DET_ACCEL_EMA_ALPHA                   (0.85f)

/* Per-sample decay of the cumulative arc while still/translating, and the
 * deadband (deg) below which it snaps to zero. */
#define DET_CUM_ARC_DECAY                     (0.97f)
#define DET_CUM_ARC_ZERO_DEG                  (5.0f)

/* The window is split into this many quarters for the yaw direction check;
 * a quarter's |yaw sum| must exceed DET_QUARTER_MIN_ABS to count. */
#define DET_QUARTER_COUNT                     (4u)
#define DET_QUARTER_MIN_ABS                   (1e-3f)

/* Small guard added to a denominator to avoid divide-by-zero. */
#define DET_DIV_EPS                           (1e-6f)

typedef struct
{
	uint8_t detected;     /* 1 if circular motion identified, else 0      */
	int8_t  direction;    /* +1 = CW, -1 = CCW, 0 = N/A                   */
	float   meanYawRateDps;
	float   deltaPsiDeg;   /* window arc (deg)                            */
	float   varX;         /* g2, linear accel X variance over the window  */
	float   varY;         /* g2, linear accel Y variance over the window  */
	/* --- Debug fields (for DIAG_TEST serial trace) --- */
	float   cumArcDeg;        /* current cumulative-arc accumulator        */
	float   varRatio;         /* max/min variance ratio                    */
	uint8_t matchingQuarters; /* how many of 4 quarters share direction    */
	uint8_t windowCandidate;  /* 1 if all per-window gates passed          */
} ts_Detection_Result;

void Detection_Init(void);

ts_Detection_Result Detection_Process(const ts_Processed_Data *sample);

#ifdef __cplusplus
}
#endif

#endif /* DETECTION_INC_DETECTION_H_ */
