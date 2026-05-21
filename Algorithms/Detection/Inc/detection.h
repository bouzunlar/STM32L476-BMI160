#ifndef DETECTION_INC_DETECTION_H_
#define DETECTION_INC_DETECTION_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "filter.h"

#define DETECTION_WINDOW_SIZE                 32u
#define DETECTION_GYRO_Z_MEAN_DPS_THRESHOLD   (120.0f)  /* mean |rotation| over window */
#define DETECTION_ACCEL_VAR_G2_THRESHOLD      (0.25f)   /* variance in g²              */

typedef struct
{
	uint8_t detected;     /* 1 if circular motion identified, else 0      */
	int8_t  direction;    /* +1 = CW, -1 = CCW, 0 = N/A                   */
	float   meanGyrZ;     /* dps — value that triggered (or current)      */
	float   varX;         /* g² — accel X variance over the window        */
	float   varY;         /* g² — accel Y variance over the window        */
} ts_Detection_Result;

void Detection_Init(void);

ts_Detection_Result Detection_Process(const ts_Processed_Data *sample);

#ifdef __cplusplus
}
#endif

#endif /* DETECTION_INC_DETECTION_H_ */
