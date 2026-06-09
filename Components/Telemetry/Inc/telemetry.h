#ifndef TELEMETRY_INC_TELEMETRY_H_
#define TELEMETRY_INC_TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "fusion.h"

/* -------------------------------------------------------------------------
 * Binary telemetry protocol
 *
 *  ┌──────┬──────┬──────┬─────────────┬─────────┬──────┐
 *  │ SYNC │ TYPE │ LEN  │  PAYLOAD    │ CRC16   │ END  │
 *  │ 0xAA │ 1B   │ 1B   │  LEN bytes  │ 2 bytes │ 0x55 │
 *  └──────┴──────┴──────┴─────────────┴─────────┴──────┘
 *
 *  CRC: CRC16-CCITT (polynomial 0x1021, init 0xFFFF) over TYPE+LEN+PAYLOAD.
 *  Multi-byte fields are little-endian (Cortex-M4 native).
 * ------------------------------------------------------------------------- */

#define TELEMETRY_SYNC            (0xAAu)
#define TELEMETRY_END             (0x55u)

#define TELEMETRY_FRAME_SENSOR    (0x01u)

#define TELEMETRY_FLAG_DETECTION  (1u << 0)

void Telemetry_Init(void);

void Telemetry_SendSensorFrame(const ts_Processed_Data *data, uint16_t flags);

/* Public CRC16-CCITT (poly 0x1021, init 0xFFFF) for on-device self-test. */
uint16_t Telemetry_Crc16Ccitt(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif



#endif /* TELEMETRY_INC_TELEMETRY_H_ */
