#include "detection.h"
#include <string.h>
#include <math.h>

static ts_Processed_Data s_window[DETECTION_WINDOW_SIZE];
static uint8_t           s_windowIndex;
static uint8_t           s_windowFull;

void Detection_Init(void)
{
	memset(s_window, 0, sizeof(s_window));
	s_windowIndex = 0u;
	s_windowFull  = 0u;
}

ts_Detection_Result Detection_Process(const ts_Processed_Data *sample)
{
	ts_Detection_Result result = { 0u, 0, 0.0f, 0.0f, 0.0f };

	if (sample == NULL)
	{
		return result;
	}

	/* 1) Push sample into the sliding window. */
	s_window[s_windowIndex] = *sample;
	s_windowIndex = (uint8_t) ((s_windowIndex + 1u) % DETECTION_WINDOW_SIZE);
	if (s_windowIndex == 0u)
	{
		s_windowFull = 1u;
	}

	if (!s_windowFull)
	{
		return result;
	}

	/* 2) Direction consistency: same gyroZ sign across all 4 quarters. */
	float gyroZSum    = 0.0f;
	float quarter[4]  = { 0.0f, 0.0f, 0.0f, 0.0f };
	for (uint8_t i = 0u; i < DETECTION_WINDOW_SIZE; i++)
	{
		gyroZSum += s_window[i].fGyrZ;
		quarter[i / (DETECTION_WINDOW_SIZE / 4u)] += s_window[i].fGyrZ;
	}

	uint8_t directionConsistent = 1u;
	int8_t  expectedSign        = (gyroZSum > 0.0f) ? 1 : -1;
	for (uint8_t q = 0u; q < 4u; q++)
	{
		int8_t qSign = (quarter[q] > 0.0f) ? 1 : -1;
		if (qSign != expectedSign)
		{
			directionConsistent = 0u;
			break;
		}
	}

	/* 3) Mean of accel X and Y over the window. */
	float meanX = 0.0f;
	float meanY = 0.0f;
	for (uint8_t i = 0u; i < DETECTION_WINDOW_SIZE; i++)
	{
		meanX += s_window[i].fAccX;
		meanY += s_window[i].fAccY;
	}
	meanX /= (float) DETECTION_WINDOW_SIZE;
	meanY /= (float) DETECTION_WINDOW_SIZE;

	/* 4) Variance, covariance, zero-crossings of accel XY. */
	float   varX  = 0.0f;
	float   varY  = 0.0f;
	float   covXY = 0.0f;
	uint8_t zcx   = 0u;
	uint8_t zcy   = 0u;

	for (uint8_t i = 0u; i < DETECTION_WINDOW_SIZE; i++)
	{
		float dx = s_window[i].fAccX - meanX;
		float dy = s_window[i].fAccY - meanY;

		varX  += dx * dx;
		varY  += dy * dy;
		covXY += dx * dy;

		if (i > 0u)
		{
			float prev_dx = s_window[i - 1u].fAccX - meanX;
			float prev_dy = s_window[i - 1u].fAccY - meanY;

			if ((prev_dx > 0.0f && dx <= 0.0f) || (prev_dx < 0.0f && dx >= 0.0f))
			{
				zcx++;
			}
			if ((prev_dy > 0.0f && dy <= 0.0f) || (prev_dy < 0.0f && dy >= 0.0f))
			{
				zcy++;
			}
		}
	}
	varX  /= (float) DETECTION_WINDOW_SIZE;
	varY  /= (float) DETECTION_WINDOW_SIZE;
	covXY /= (float) DETECTION_WINDOW_SIZE;

	const uint8_t notStraightLine = ((covXY * covXY) < (varX * varY * 0.5f));
	const uint8_t isFullCircle    = (zcx >= 2u) && (zcy >= 2u);
	const float   meanGyrZ        = fabsf(gyroZSum) / (float) DETECTION_WINDOW_SIZE;

	result.meanGyrZ = meanGyrZ;
	result.varX     = varX;
	result.varY     = varY;

	if (directionConsistent &&
	    notStraightLine &&
	    isFullCircle &&
	    meanGyrZ > DETECTION_GYRO_Z_MEAN_DPS_THRESHOLD &&
	    varX     > DETECTION_ACCEL_VAR_G2_THRESHOLD &&
	    varY     > DETECTION_ACCEL_VAR_G2_THRESHOLD)
	{
		result.detected  = 1u;
		result.direction = (gyroZSum > 0.0f) ? 1 : -1;

		/* Reset window so the next detection starts clean. */
		memset(s_window, 0, sizeof(s_window));
		s_windowIndex = 0u;
		s_windowFull  = 0u;
	}

	return result;
}
