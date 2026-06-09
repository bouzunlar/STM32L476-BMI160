#include "fusion.h"
#include <math.h>

/* -------------------------------------------------------------------------
 * Mahony 6-axis PI-controller fusion filter (IMU, no magnetometer).
 *
 * Idea — treat the gyro as a process and use the accelerometer as a slow
 * reference. At every step:
 *
 *   1. Compute the expected gravity direction in the body frame from the
 *      current quaternion (rotate (0,0,1) by q).
 *   2. The "error" is the cross product between the accel-measured
 *      gravity and the expected gravity. This vector points along the
 *      rotation axis that would align them.
 *   3. Feed the error into a PI controller and add it to the gyro
 *      reading. The integral term naturally converges to the gyro bias
 *      — no separate calibration step needed.
 *   4. Integrate the corrected gyro to update the quaternion.
 *
 * Reference:
 *   Mahony, R. et al. (2008). "Nonlinear complementary filters on the
 *   special orthogonal group."
 * ------------------------------------------------------------------------- */

#define DEG2RAD     (0.017453293f)
#define FUSION_ACCEL_MIN_G      (0.20f)
#define FUSION_ACCEL_MAX_G      (3.50f)
#define FUSION_ACCEL_CORR_MIN_G (0.70f)
#define FUSION_ACCEL_CORR_MAX_G (1.30f)
#define FUSION_GYRO_MAX_DPS     (480.0f)

/* Quaternion state — identity (no rotation) at startup. */
static float    s_q0 = 1.0f, s_q1 = 0.0f, s_q2 = 0.0f, s_q3 = 0.0f;

/* PI integral term — converges to gyro bias (rad/s). */
static float    s_intEx, s_intEy, s_intEz;

/* Cumulative yaw — free-running gyro Z integration for circle detection. */
static float    s_yawCum;

/* Monotonic sequence number. */
static uint32_t s_seq;

static uint8_t  s_init;

static void fusion_get_gravity_body(float *gx, float *gy, float *gz)
{
	if (gx != NULL)
	{
		*gx = 2.0f * (s_q1 * s_q3 - s_q0 * s_q2);
	}
	if (gy != NULL)
	{
		*gy = 2.0f * (s_q0 * s_q1 + s_q2 * s_q3);
	}
	if (gz != NULL)
	{
		*gz = s_q0 * s_q0 - s_q1 * s_q1 - s_q2 * s_q2 + s_q3 * s_q3;
	}
}

static void fusion_rotate_body_to_earth(float bx, float by, float bz,
                                        float *ex, float *ey, float *ez)
{
	const float tx = 2.0f * (s_q2 * bz - s_q3 * by);
	const float ty = 2.0f * (s_q3 * bx - s_q1 * bz);
	const float tz = 2.0f * (s_q1 * by - s_q2 * bx);

	if (ex != NULL)
	{
		*ex = bx + s_q0 * tx + s_q2 * tz - s_q3 * ty;
	}
	if (ey != NULL)
	{
		*ey = by + s_q0 * ty + s_q3 * tx - s_q1 * tz;
	}
	if (ez != NULL)
	{
		*ez = bz + s_q0 * tz + s_q1 * ty - s_q2 * tx;
	}
}

void Fusion_Init(void)
{
	s_q0     = 1.0f;
	s_q1     = 0.0f;
	s_q2     = 0.0f;
	s_q3     = 0.0f;
	s_intEx  = 0.0f;
	s_intEy  = 0.0f;
	s_intEz  = 0.0f;
	s_yawCum = 0.0f;
	s_seq    = 0u;
	s_init   = 0u;
}

void Fusion_ResetYaw(void)
{
	s_yawCum = 0.0f;
}

void Fusion_Apply(const ts_Bmi160_Data *in, ts_Processed_Data *out)
{
	if (in == NULL || out == NULL)
	{
		return;
	}

	/* Inputs: accel in g, gyro in dps and rad/s. */
	const float rawAx = in->scaledAccX;
	const float rawAy = in->scaledAccY;
	const float rawAz = in->scaledAccZ;
	const float gyroXDps = in->scaledGyrX;
	const float gyroYDps = in->scaledGyrY;
	const float gyroZDps = in->scaledGyrZ;

	float ax = rawAx;
	float ay = rawAy;
	float az = rawAz;
	float gx = gyroXDps * DEG2RAD;
	float gy = gyroYDps * DEG2RAD;
	float gz = gyroZDps * DEG2RAD;

	const float accel_sq = ax * ax + ay * ay + az * az;
	const float accelMag = sqrtf(accel_sq);
	const uint8_t accelUsable = (accelMag >= FUSION_ACCEL_MIN_G) &&
	                            (accelMag <= FUSION_ACCEL_MAX_G);
	const uint8_t accelCorrectionUsable = (accelMag >= FUSION_ACCEL_CORR_MIN_G) &&
	                                      (accelMag <= FUSION_ACCEL_CORR_MAX_G);
	const uint8_t gyroUsable = (fabsf(gyroXDps) <= FUSION_GYRO_MAX_DPS) &&
	                           (fabsf(gyroYDps) <= FUSION_GYRO_MAX_DPS) &&
	                           (fabsf(gyroZDps) <= FUSION_GYRO_MAX_DPS);
	uint8_t fusionHealthy = (accelUsable && gyroUsable) ? 1u : 0u;

	/* Apply accel correction only when acceleration is close to 1 g.
	 * During dynamic motion, gyro integration continues without PI correction. */
	if (gyroUsable && accelCorrectionUsable && accel_sq > 1e-6f)
	{
		/* Normalise accelerometer to a unit vector. */
		const float recipNorm = 1.0f / accelMag;
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		float vx, vy, vz;
		fusion_get_gravity_body(&vx, &vy, &vz);

		/* Error = measured × predicted (cross product). Points along the
		 * rotation axis that would align the two. */
		const float ex = (ay * vz) - (az * vy);
		const float ey = (az * vx) - (ax * vz);
		const float ez = (ax * vy) - (ay * vx);

		/* Integral term — slowly accumulates the gyro bias. */
		s_intEx += FUSION_MAHONY_KI * ex * IMU_DT_SEC;
		s_intEy += FUSION_MAHONY_KI * ey * IMU_DT_SEC;
		s_intEz += FUSION_MAHONY_KI * ez * IMU_DT_SEC;

		/* PI feedback on the gyro rate. */
		gx += FUSION_MAHONY_KP * ex + s_intEx;
		gy += FUSION_MAHONY_KP * ey + s_intEy;
		gz += FUSION_MAHONY_KP * ez + s_intEz;
	}

	if (gyroUsable)
	{
	/* Quaternion derivative from corrected gyro rate. */
	const float qa = s_q0;
	const float qb = s_q1;
	const float qc = s_q2;
	const float qd = s_q3;

	s_q0 += 0.5f * (-qb * gx - qc * gy - qd * gz) * IMU_DT_SEC;
	s_q1 += 0.5f * ( qa * gx + qc * gz - qd * gy) * IMU_DT_SEC;
	s_q2 += 0.5f * ( qa * gy - qb * gz + qd * gx) * IMU_DT_SEC;
	s_q3 += 0.5f * ( qa * gz + qb * gy - qc * gx) * IMU_DT_SEC;
	}

	/* Renormalise to a unit quaternion. */
	const float qnorm = sqrtf(s_q0 * s_q0 + s_q1 * s_q1 + s_q2 * s_q2 + s_q3 * s_q3);
	if (qnorm > 1e-9f)
	{
		const float inv = 1.0f / qnorm;
		s_q0 *= inv;
		s_q1 *= inv;
		s_q2 *= inv;
		s_q3 *= inv;
	}
	else
	{
		s_q0 = 1.0f;
		s_q1 = 0.0f;
		s_q2 = 0.0f;
		s_q3 = 0.0f;
		fusionHealthy = 0u;
	}

	float gBodyX, gBodyY, gBodyZ;
	fusion_get_gravity_body(&gBodyX, &gBodyY, &gBodyZ);
	const float gNorm = sqrtf(gBodyX * gBodyX + gBodyY * gBodyY + gBodyZ * gBodyZ);
	if (gNorm > 1e-6f)
	{
		const float invG = 1.0f / gNorm;
		gBodyX *= invG;
		gBodyY *= invG;
		gBodyZ *= invG;
	}

	const float yawRateDps = gyroXDps * gBodyX + gyroYDps * gBodyY + gyroZDps * gBodyZ;
	s_yawCum += yawRateDps * IMU_DT_SEC;

	float aEarthX, aEarthY, aEarthZ;
	fusion_rotate_body_to_earth(rawAx, rawAy, rawAz, &aEarthX, &aEarthY, &aEarthZ);

	if (!s_init)
	{
		s_init = 1u;
	}

	/* Output frame — scaled passthrough + fused orientation. */
	out->raw    = *in;
	out->fAccX  = aEarthX;
	out->fAccY  = aEarthY;
	out->fAccZ  = aEarthZ - FUSION_GRAVITY_G;
	out->fGyrX  = gyroXDps;
	out->fGyrY  = gyroYDps;
	out->fGyrZ  = gyroZDps;
	out->q0     = s_q0;
	out->q1     = s_q1;
	out->q2     = s_q2;
	out->q3     = s_q3;
	out->yawRateDps = yawRateDps;
	out->yawCum = s_yawCum;
	out->fusionHealthy = fusionHealthy;
	out->seq    = s_seq++;
}
