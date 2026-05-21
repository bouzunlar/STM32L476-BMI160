#include "filter.h"

static float    s_aX, s_aY, s_aZ;     /* accel EMA states [g]   */
static float    s_gX, s_gY, s_gZ;     /* gyro  EMA states [dps] */
static uint8_t  s_init;
static uint32_t s_seq;

void Filter_Init(void)
{
	s_aX = 0.0f; s_aY = 0.0f; s_aZ = 0.0f;
	s_gX = 0.0f; s_gY = 0.0f; s_gZ = 0.0f;
	s_init = 0u;
	s_seq  = 0u;
}

void Filter_Apply(const ts_Bmi160_Data *in, ts_Processed_Data *out)
{
	if (in == NULL || out == NULL)
	{
		return;
	}

	if (!s_init)
	{
		/* Seed the EMA with the first sample to avoid a ramp from zero. */
		s_aX = in->scaledAccX;
		s_aY = in->scaledAccY;
		s_aZ = in->scaledAccZ;
		s_gX = in->scaledGyrX;
		s_gY = in->scaledGyrY;
		s_gZ = in->scaledGyrZ;
		s_init = 1u;
	}
	else
	{
		const float a = FILTER_EMA_ALPHA;
		const float b = 1.0f - FILTER_EMA_ALPHA;

		s_aX = a * in->scaledAccX + b * s_aX;
		s_aY = a * in->scaledAccY + b * s_aY;
		s_aZ = a * in->scaledAccZ + b * s_aZ;
		s_gX = a * in->scaledGyrX + b * s_gX;
		s_gY = a * in->scaledGyrY + b * s_gY;
		s_gZ = a * in->scaledGyrZ + b * s_gZ;
	}

	out->raw   = *in;
	out->fAccX = s_aX;
	out->fAccY = s_aY;
	out->fAccZ = s_aZ;
	out->fGyrX = s_gX;
	out->fGyrY = s_gY;
	out->fGyrZ = s_gZ;
	out->seq   = s_seq++;
}
