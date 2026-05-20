

#ifndef DRIVER_SERIAL_INC_SERIAL_HAL_H_
#define DRIVER_SERIAL_INC_SERIAL_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <../../Board/board.h>

static const GPIO_TypeDef* serialPortList[SERIAL_TOTAL_NUM]=SERIAL_PORT_LIST;
static const uint16_t serialPinList[SERIAL_TOTAL_NUM]=SERIAL_PIN_LIST;
static const uint32_t serialFunctionList[SERIAL_TOTAL_NUM]=SERIAL_FUNC_LIST;
static const uint32_t serialPullList[SERIAL_TOTAL_NUM]=SERIAL_PULL_LIST;
static const uint32_t serialPinSpeedList[SERIAL_TOTAL_NUM]=SERIAL_PIN_SPEED_LIST;

static const USART_TypeDef* serialUsartList[SERIAL_TOTAL_NUM]=SERIAL_USART_LIST;
static const uint32_t serialBaudRateList[SERIAL_TOTAL_NUM]=SERIAL_USART_BAUDRATE_LIST;

extern DMA_HandleTypeDef hdma_usart2_tx;
static DMA_HandleTypeDef * const serialDmaHandleList[SERIAL_TOTAL_NUM] = SERIAL_DMA_HANDLE_LIST;

static const DMA_Channel_TypeDef* serialDmaInstanceList[SERIAL_TOTAL_NUM] = SERIAL_DMA_INSTANCE_LIST;
static const uint32_t serialDmaRequestList[SERIAL_TOTAL_NUM]              = SERIAL_DMA_REQUEST_LIST;
static const IRQn_Type serialDmaIrqList[SERIAL_TOTAL_NUM]                 = SERIAL_DMA_IRQ_LIST;
static const uint32_t serialAfList[SERIAL_TOTAL_NUM] = SERIAL_AF_LIST;
static const IRQn_Type serialUartIrqList[SERIAL_TOTAL_NUM]             = SERIAL_UART_IRQ_LIST;

#ifdef __cplusplus
}
#endif


#endif /* DRIVER_SERIAL_INC_SERIAL_HAL_H_ */
