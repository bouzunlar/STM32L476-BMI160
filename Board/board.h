#ifndef BOARD_H_
#define BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdio.h>
#include <stm32l476xx.h>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_gpio.h>

#define DEBUGOUT(fmt, ...) do {printf(fmt, ##__VA_ARGS__); } while(0)

#define delayMs(ms) do { \
	volatile uint32_t _dummyCnt = 0; \
    for (uint32_t _i = 0; _i < (ms); ++_i) { \
    	for (uint32_t _i = 0; _i < (1000); ++_i){_dummyCnt++;}\
    } \
} while(0)

/// Serials
#define SERIAL_TOTAL_NUM 1
#define USER_SERIAL_INDEX	0
#define SERIAL_PORT_LIST  	{GPIOA}
#define SERIAL_FUNC_LIST 	{GPIO_MODE_AF_PP}
#define SERIAL_PIN_LIST  	{GPIO_PIN_2|GPIO_PIN_3}
#define SERIAL_PULL_LIST 	{GPIO_NOPULL}
#define SERIAL_PIN_SPEED_LIST	{GPIO_SPEED_FREQ_HIGH}

#define serialClockEnable() 	do{__HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_USART2_CLK_ENABLE();}while(0)
#define serialClockDisable() 	do{__HAL_RCC_GPIOA_CLK_DISABLE();__HAL_RCC_USART2_CLK_DISABLE();}while(0)

#define SERIAL_USART_LIST 			{USART2}
#define SERIAL_USART_BAUDRATE_LIST	{115200}

#define SERIAL_MAX_INTERRUPT_HANDLERS 8


#endif /* BOARD_H_ */
