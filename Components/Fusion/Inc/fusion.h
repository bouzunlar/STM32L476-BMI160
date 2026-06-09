#ifndef FUSION_INC_FUSION_H_
#define FUSION_INC_FUSION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bmi160_driver.h"
#include "imu_timing.h"

/* -------------------------------------------------------------------------
 * Sensor fusion (gyro + accel) — orientation estimate.
 *
 * Implementation: Mahony 6-axis PI-controller filter.
 *
 *   • Kp drives proportional correction toward gravity from the accel.
 *   • Ki integrates the error → converges to the gyro bias (no separate
 *     calibration step). Hold the device still ~2 s after boot.
 *
 * The Filter / EMA stage was removed deliberately: the fusion filter
 * already provides the only smoothing needed, and double low-pass
 * filtering before fusion biases the gyro/accel match.
 *
 * Sample period dt is derived from the BMI160 ODR in imu_timing.h.
 *
 * Alternative backends (Complementary, Madgwick) lived alongside this
 * one earlier and may be reintroduced for comparison in a later phase.
 * ------------------------------------------------------------------------- */

#define FUSION_GRAVITY_G        (1.0f)
#define FUSION_MAHONY_KP        (2.0f)
#define FUSION_MAHONY_KI        (0.005f)

/* -------------------------------------------------------------------------
 * Output of the processing stage.
 *
 * Field names keep the historical f-prefix for compatibility with the
 * existing detection / telemetry code. Accel fields now hold earth-frame
 * linear acceleration after gravity removal.
 *
 *   • raw                 — original sensor frame (raw LSB + scaled)
 *   • fAccX/Y/Z [g]       — earth-frame linear acceleration
 *   • fGyrX/Y/Z [dps]     — scaled gyro passthrough
 *   • q0..q3              — fused orientation as a unit quaternion
 *   • yawRateDps          — gyro projected onto the gravity direction
 *   • yawCum [deg]        — free-running cumulative yaw integration
 *                           from yawRateDps
 *   • fusionHealthy       — 1 when the current fusion sample is usable
 *   • seq                 — monotonic sequence number
 * ------------------------------------------------------------------------- */
typedef struct
{
	ts_Bmi160_Data raw;

	float          fAccX;      /* g  */
	float          fAccY;
	float          fAccZ;
	float          fGyrX;      /* dps */
	float          fGyrY;
	float          fGyrZ;

	float          q0;         /* unit quaternion (w, x, y, z) */
	float          q1;
	float          q2;
	float          q3;

	float          yawRateDps; /* dps, rotation about estimated gravity */
	float          yawCum;     /* deg, cumulative — not wrapped at ±180° */
	uint8_t        fusionHealthy;

	uint32_t       seq;
} ts_Processed_Data;

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
void Fusion_Init(void);

/* Consume one raw sensor frame, produce one processed frame
 * (orientation quaternion + cumulative yaw + scaled passthrough). */
void Fusion_Apply(const ts_Bmi160_Data *in, ts_Processed_Data *out);

/* Reset cumulative yaw to zero (e.g. after a detected circle). */
void Fusion_ResetYaw(void);

/* Quaternion → Euler (deg). Implementation-independent helper. Pass NULL
 * for any angle you don't need. */
void Fusion_QuatToEuler(float q0, float q1, float q2, float q3,
                        float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif /* FUSION_INC_FUSION_H_ */
