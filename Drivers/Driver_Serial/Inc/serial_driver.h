#ifndef DRIVER_SERIAL_INC_SERIAL_DRIVER_H_
#define DRIVER_SERIAL_INC_SERIAL_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#define SERIAL_MODULE_SW_VERSION	(1.0)

#include <board.h>

typedef enum
{
	E_SERIAL_ERR_NONE,
	E_SERIAL_ERR_WRONG_INDEX,
	E_SERIAL_ERR_HW_ERROR,
	E_SERIAL_ERR_WRONG_IOCTL_CMD,
	E_SERIAL_ERR_DMA_NOT_ENABLED,
	E_SERIAL_ERR_UNKNOWN
} serialErrorCodes_t;

typedef enum
{
	E_SERIAL_IOCTL_NONE,
	E_SERIAL_IOCTL_GET_VERSION,
	E_SERIAL_IOCTL_ENABLE_INTERRUPT,
	E_SERIAL_IOCTL_DISABLE_INTERRUPT,
	E_SERIAL_IOCTL_REGISTER_IRQ_HANDLER,
	E_SERIAL_IOCTL_ENABLE_DMA_TX
} SERIAL_IOCTL_COMMANDS_T;

typedef struct
{
	uint8_t index;
	uint8_t *ptr;
	uint16_t length;
} SerialReadWrite_t;

typedef void (*serial_irq_handler_t)(void*);

typedef struct
{
	uint8_t index;
	uint8_t isInterruptOccured;
	uint32_t type;
	uint32_t interrupt_id;
	serial_irq_handler_t interrupt_handler;
} serialInterrupt_t;

extern serialInterrupt_t serial_irq_context[SERIAL_MAX_INTERRUPT_HANDLERS];

serialErrorCodes_t Serial_Open(void *vpParam);

serialErrorCodes_t Serial_Ioctl(SERIAL_IOCTL_COMMANDS_T eCommand, void *vpParam);

serialErrorCodes_t Serial_Write_DMA(const void *pvBuffer, const uint32_t xBytes);

serialErrorCodes_t Serial_Write(const void *pvBuffer, const uint32_t xBytes);


serialErrorCodes_t Serial_Read(const void *pvBuffer, const uint32_t xBytes);

serialErrorCodes_t Serial_Close(void *vpParam);

void Serial_DMA_TX_IRQHandler(uint8_t index);

int8_t SERIAL_TEST(uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_SERIAL_INC_SERIAL_DRIVER_H_ */
