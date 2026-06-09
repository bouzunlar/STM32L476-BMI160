#include "fusion.h"
#include <math.h>

/* -------------------------------------------------------------------------
 * Implementation-independent helpers for the Fusion module.
 *
 * Lives in a separate translation unit (no #if guards) so it compiles in
 * regardless of which Fusion algorithm is selected, and there's one
 * canonical conversion routine for the whole project.
 * ------------------------------------------------------------------------- */

#define RAD2DEG    (57.295779513f)

void Fusion_QuatToEuler(float q0, float q1, float q2, float q3,
                        float *roll, float *pitch, float *yaw)
{
	if (roll != NULL)
	{
		*roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
		               1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
	}

	if (pitch != NULL)
	{
		float sinp = 2.0f * (q0 * q2 - q3 * q1);
		if (sinp >  1.0f) sinp =  1.0f;
		if (sinp < -1.0f) sinp = -1.0f;
		*pitch = asinf(sinp) * RAD2DEG;
	}

	if (yaw != NULL)
	{
		*yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
		              1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
	}
}
