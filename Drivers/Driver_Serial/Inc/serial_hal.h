

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


#ifdef __cplusplus
}
#endif


#endif /* DRIVER_SERIAL_INC_SERIAL_HAL_H_ */
