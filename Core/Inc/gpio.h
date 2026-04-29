/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
typedef enum {
    DIR_IN = 0,
    DIR_OUT = 1
} te_Gpio_Direction;

typedef enum {
    GPIO_MODE_PLAIN = 0,
    GPIO_MODE_PULL_UP = 1,
    GPIO_MODE_PULL_DOWN = 2
} te_Gpio_Mode;
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
void Gpio_SetDirection(uint32_t port, uint16_t pin, te_Gpio_Direction dir);
void Gpio_SetPinImpedanceMode(uint32_t port, uint16_t pin, te_Gpio_Mode mode);
void Gpio_SelectFunction(uint32_t port, uint16_t pin, uint8_t func);
void Gpio_SetAnalogPinMode(uint32_t port, uint16_t pin, uint8_t enable);
void Gpio_EnableInterrupt(uint32_t port, uint16_t pin, uint8_t edge);
void Gpio_IRQ_Init(uint32_t port, uint16_t pin, void (*handler)(uint32_t, uint8_t), uint32_t id);
void Gpio_IRQ_Enable(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

