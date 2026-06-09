#include "telemetry.h"
#include "uart_tx.h"
#include <string.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Sensor frame payload (67 bytes, little-endian):
 *
 *   offset  size  field
 *   ──────  ────  ─────────────────────────────────────────
 *      0     4    uint32  seq
 *      4     6    int16   rawAccX, rawAccY, rawAccZ
 *     10     6    int16   rawGyrX, rawGyrY, rawGyrZ
 *     16    12    float   fAccX,   fAccY,   fAccZ        [g, linear earth]
 *     28    12    float   fGyrX,   fGyrY,   fGyrZ        [dps]
 *     40    16    float   q0, q1, q2, q3                 (unit quaternion)
 *     56     4    float   yawRateDps                     [dps]
 *     60     4    float   yawCum                         [deg, cumulative]
 *     64     1    uint8   fusionHealthy
 *     65     2    uint16  flags
 *
 * Frame total = 1 (SYNC) + 1 (TYPE) + 1 (LEN) + 67 + 2 (CRC) + 1 (END) = 73 B
 * Bandwidth   = 100 Hz x 73 B = 7.3 kB/s (about 63 % of 115200 bps)
 * ------------------------------------------------------------------------- */

#define PAYLOAD_SIZE_SENSOR   (67u)
#define FRAME_SIZE_SENSOR     (PAYLOAD_SIZE_SENSOR + 6u)   /* 73 */

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
	uint16_t crc = 0xFFFFu;
	for (uint16_t i = 0u; i < len; i++)
	{
		crc ^= ((uint16_t) data[i]) << 8;
		for (uint8_t j = 0u; j < 8u; j++)
		{
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

uint16_t Telemetry_Crc16Ccitt(const uint8_t *data, uint16_t len)
{
	return crc16_ccitt(data, len);
}

void Telemetry_Init(void)
{
	/* Reserved for future on/off / config */
}

void Telemetry_SendSensorFrame(const ts_Processed_Data *data, uint16_t flags)
{
	if (data == NULL)
	{
		return;
	}

	uint8_t  buf[FRAME_SIZE_SENSOR];
	uint16_t idx = 0u;

	/* Header */
	buf[idx++] = TELEMETRY_SYNC;
	buf[idx++] = TELEMETRY_FRAME_SENSOR;
	buf[idx++] = (uint8_t) PAYLOAD_SIZE_SENSOR;

	/* Payload */
	memcpy(&buf[idx], &data->seq,             4u); idx += 4u;
	memcpy(&buf[idx], &data->raw.rawAccX,     2u); idx += 2u;
	memcpy(&buf[idx], &data->raw.rawAccY,     2u); idx += 2u;
	memcpy(&buf[idx], &data->raw.rawAccZ,     2u); idx += 2u;
	memcpy(&buf[idx], &data->raw.rawGyrX,     2u); idx += 2u;
	memcpy(&buf[idx], &data->raw.rawGyrY,     2u); idx += 2u;
	memcpy(&buf[idx], &data->raw.rawGyrZ,     2u); idx += 2u;
	memcpy(&buf[idx], &data->fAccX,           4u); idx += 4u;
	memcpy(&buf[idx], &data->fAccY,           4u); idx += 4u;
	memcpy(&buf[idx], &data->fAccZ,           4u); idx += 4u;
	memcpy(&buf[idx], &data->fGyrX,           4u); idx += 4u;
	memcpy(&buf[idx], &data->fGyrY,           4u); idx += 4u;
	memcpy(&buf[idx], &data->fGyrZ,           4u); idx += 4u;
	memcpy(&buf[idx], &data->q0,              4u); idx += 4u;
	memcpy(&buf[idx], &data->q1,              4u); idx += 4u;
	memcpy(&buf[idx], &data->q2,              4u); idx += 4u;
	memcpy(&buf[idx], &data->q3,              4u); idx += 4u;
	memcpy(&buf[idx], &data->yawRateDps,      4u); idx += 4u;
	memcpy(&buf[idx], &data->yawCum,          4u); idx += 4u;
	buf[idx++] = data->fusionHealthy;
	memcpy(&buf[idx], &flags,                 2u); idx += 2u;

	/* CRC over TYPE + LEN + PAYLOAD (everything since byte 1). */
	uint16_t crc = crc16_ccitt(&buf[1], idx - 1u);
	buf[idx++] = (uint8_t) (crc >> 8);
	buf[idx++] = (uint8_t) (crc & 0xFFu);

	/* Trailer */
	buf[idx++] = TELEMETRY_END;

	UartTx_SendBytes(buf, idx);
}
