#ifndef UARTTX_INC_UART_TX_H_
#define UARTTX_INC_UART_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#define UART_TX_MSG_MAX     96u

void UartTx_Init(void);

void UartTx_SendBytes(const uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif


#endif /* UARTTX_INC_UART_TX_H_ */
