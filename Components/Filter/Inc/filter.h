#ifndef FILTER_INC_FILTER_H_
#define FILTER_INC_FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bmi160_driver.h"

#define FILTER_EMA_ALPHA   (0.20f)   /* ≈ 3.5 Hz cutoff @ 100 Hz sampling */

typedef struct
{
	ts_Bmi160_Data raw;         /* original sensor frame (raw + scaled)   */
	float          fAccX;       /* filtered accel X [g]                   */
	float          fAccY;
	float          fAccZ;
	float          fGyrX;       /* filtered gyro  X [dps]                 */
	float          fGyrY;
	float          fGyrZ;
	uint32_t       seq;         /* monotonic sequence number              */
} ts_Processed_Data;

void Filter_Init(void);

void Filter_Apply(const ts_Bmi160_Data *in, ts_Processed_Data *out);

#ifdef __cplusplus
}
#endif


#endif /* FILTER_INC_FILTER_H_ */
