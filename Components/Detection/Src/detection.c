#include "detection.h"
#include <string.h>
#include <math.h>

static ts_Processed_Data s_window[DETECTION_WINDOW_SIZE];
static uint8_t           s_windowIndex;
static uint8_t           s_windowFull;

/* Integrates the rotation of the centripetal accel vector */
static float  s_cumArcDeg;
static float   s_lastAccelAngle;   /* previous atan2(fAccY, fAccX), rad */
static uint8_t s_accelAngleValid;  /* 0 until first trustworthy angle */
static float   s_accSmoothX;       /* EMA-smoothed fAccX/Y to de-jitter angle */
static float   s_accSmoothY;

void Detection_Init(void)
{
	memset(s_window, 0, sizeof(s_window));
	s_windowIndex   = 0u;
	s_windowFull    = 0u;
	s_cumArcDeg     = 0.0f;
	s_lastAccelAngle  = 0.0f;
	s_accelAngleValid = 0u;
	s_accSmoothX      = 0.0f;
	s_accSmoothY      = 0.0f;
}

ts_Detection_Result Detection_Process(const ts_Processed_Data *sample)
{
	ts_Detection_Result result = { 0u, 0, 0.0f, 0.0f, 0.0f, 0.0f };

	if (sample == NULL)
	{
		return result;
	}

	if (sample->fusionHealthy == 0u)
	{
		/* Pause accumulation but keep the window so motion resumes instantly. */
		s_accelAngleValid = 0u;
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

	/* 2) Yaw sums: signed (direction) + absolute (wrist activity). */
	float yawRateSum = 0.0f;
	float sumAbsYaw  = 0.0f;
	float quarter[DET_QUARTER_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };
	for (uint8_t i = 0u; i < DETECTION_WINDOW_SIZE; i++)
	{
		yawRateSum += s_window[i].yawRateDps;
		sumAbsYaw  += fabsf(s_window[i].yawRateDps);
		quarter[i / (DETECTION_WINDOW_SIZE / DET_QUARTER_COUNT)] += s_window[i].yawRateDps;
	}
	const float meanAbsYaw = sumAbsYaw / (float) DETECTION_WINDOW_SIZE;

	const int8_t expectedSign = (yawRateSum >= 0.0f) ? 1 : -1;
	uint8_t matchingQuarters = 0u;
	if (DET_YAW_QUARTERS_REQUIRED == 0u)
	{
		matchingQuarters = DET_QUARTER_COUNT;
	}
	else
	{
	for (uint8_t q = 0u; q < DET_QUARTER_COUNT; q++)
	{
			if (fabsf(quarter[q]) > DET_QUARTER_MIN_ABS)
		{
				const int8_t qSign = (quarter[q] >= 0.0f) ? 1 : -1;
				if (qSign == expectedSign)
				{
					matchingQuarters++;
		}
	}
		}
	}
	const uint8_t directionConsistent = (matchingQuarters >= DET_YAW_QUARTERS_REQUIRED);

	/* 3) Mean of earth-frame linear accel X and Y over the window. */
	float meanX = 0.0f;
	float meanY = 0.0f;
	for (uint8_t i = 0u; i < DETECTION_WINDOW_SIZE; i++)
	{
		meanX += s_window[i].fAccX;
		meanY += s_window[i].fAccY;
	}
	meanX /= (float) DETECTION_WINDOW_SIZE;
	meanY /= (float) DETECTION_WINDOW_SIZE;

	/* 4) Variance, covariance, zero-crossings of linear accel XY. */
	float   varX     = 0.0f;
	float   varY     = 0.0f;
	float   covXY    = 0.0f;
	uint8_t zcx      = 0u;
	uint8_t zcy      = 0u;

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
	varX     /= (float) DETECTION_WINDOW_SIZE;
	varY     /= (float) DETECTION_WINDOW_SIZE;
	covXY    /= (float) DETECTION_WINDOW_SIZE;

	float deltaPsiDeg = 0.0f;
	for (uint8_t i = 0u; i < DETECTION_WINDOW_SIZE; i++)
	{
		deltaPsiDeg += s_window[i].yawRateDps * IMU_DT_SEC;
	}

	const float minVar = (varX < varY) ? varX : varY;
	const float maxVar = (varX > varY) ? varX : varY;
	const float varRatio = maxVar / (minVar + DET_DIV_EPS);
	const uint8_t shapeOK = (varRatio <= DET_VAR_RATIO_MAX);
	const uint8_t notStraightLine = ((covXY * covXY) < (varX * varY * DET_COV_K_FACTOR));
	const uint8_t zcOK = (zcx >= DET_MIN_ZERO_CROSS_EACH_AXIS) &&
	                     (zcy >= DET_MIN_ZERO_CROSS_EACH_AXIS);
	const uint8_t arcOK = (fabsf(deltaPsiDeg) >= DET_MIN_ARC_DEG);
	const uint8_t phaseOK = (DET_MIN_ZERO_CROSS_EACH_AXIS == 0u) ? arcOK : (zcOK || arcOK);
	const float meanYawRate = fabsf(yawRateSum) / (float) DETECTION_WINDOW_SIZE;

	result.meanYawRateDps = meanYawRate;
	result.deltaPsiDeg    = deltaPsiDeg;
	result.varX     = varX;
	result.varY     = varY;
	result.varRatio = varRatio;
	result.matchingQuarters = matchingQuarters;

	/* Candidate = linear-accel pattern looks circular now (2D orbit, balanced
	 * axes, bounded energy). Full-turn confirmation is the accel arc below. */
	const uint8_t windowCandidate = (notStraightLine &&
	                                  shapeOK &&
	                                  varX        > DET_ALIN_VAR_G2_MIN &&
	                                  varY > DET_ALIN_VAR_G2_MIN &&
	                                  varX < DET_ALIN_VAR_G2_MAX &&
	                                  varY < DET_ALIN_VAR_G2_MAX);
	result.windowCandidate = windowCandidate;
	(void) directionConsistent;   /* debug trace only */
	(void) phaseOK;
	(void) meanYawRate;

	/* Accumulate arc only during real circular motion (bounded energy + wrist
	 * rotating); the upper var bound blocks fast/violent half-arcs. */
	const uint8_t enoughMotion = ((varX + varY) > (2.0f * DET_ALIN_VAR_G2_MIN)) &&
	                             (varX < DET_ALIN_VAR_G2_MAX) &&
	                             (varY < DET_ALIN_VAR_G2_MAX);
	const uint8_t wristActive  = (meanAbsYaw > DET_YAW_ABS_MEAN_MIN_DPS);
	/* EMA-smooth the accel vector; raw signal is too jittery and inflates
	 * the integrated angle. */
	s_accSmoothX = DET_ACCEL_EMA_ALPHA * s_accSmoothX +
	               (1.0f - DET_ACCEL_EMA_ALPHA) * sample->fAccX;
	s_accSmoothY = DET_ACCEL_EMA_ALPHA * s_accSmoothY +
	               (1.0f - DET_ACCEL_EMA_ALPHA) * sample->fAccY;
	const float aMagSmooth = sqrtf(s_accSmoothX * s_accSmoothX +
	                               s_accSmoothY * s_accSmoothY);

	if (enoughMotion && wristActive && aMagSmooth > DET_ALIN_ANGLE_MIN_G)
	{
		const float angle = atan2f(s_accSmoothY, s_accSmoothX);  /* rad */
		if (s_accelAngleValid)
		{
			float dAng = angle - s_lastAccelAngle;
			if (dAng >  DET_PI_RAD) { dAng -= DET_TWO_PI_RAD; }   /* unwrap */
			else if (dAng < -DET_PI_RAD) { dAng += DET_TWO_PI_RAD; }
			s_cumArcDeg += dAng * DET_RAD_TO_DEG;   /* rad -> deg, signed */
	}
		s_lastAccelAngle  = angle;
		s_accelAngleValid = 1u;
	}
	else
	{
		/* Still/translation: skip the angle and bleed off old arc. */
		s_accelAngleValid = 0u;
		s_cumArcDeg *= DET_CUM_ARC_DECAY;
		if (fabsf(s_cumArcDeg) < DET_CUM_ARC_ZERO_DEG) { s_cumArcDeg = 0.0f; }
	}

	result.cumArcDeg = s_cumArcDeg;   /* report before any reset (debug) */

	/* A full accel-vector revolution = a circle. Fire and reset. */
	if (fabsf(s_cumArcDeg) >= DET_CUM_ARC_THRESHOLD_DEG)
	{
		result.detected  = 1u;
		result.direction = (s_cumArcDeg >= 0.0f) ? 1 : -1;

		/* Reset arc only; keep the window so back-to-back circles aren't lost. */
		s_cumArcDeg     = 0.0f;
		s_accelAngleValid = 0u;
	}

	return result;
}
