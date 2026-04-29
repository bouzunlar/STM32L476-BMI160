/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMI160_INT_Pin */
  GPIO_InitStruct.Pin = BMI160_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BMI160_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
void Gpio_SetDirection(uint32_t port, uint16_t pin, te_Gpio_Direction dir) {
    // CubeMX zaten MX_GPIO_Init içinde yönü belirliyor, boş kalabilir.
}

void Gpio_SetPinImpedanceMode(uint32_t port, uint16_t pin, te_Gpio_Mode mode) {
    // Pull-up / Pull-down ayarları gerekirse burada HAL_GPIO_Init ile yapılabilir.
}

void Gpio_SelectFunction(uint32_t port, uint16_t pin, uint8_t func) {
    // Alternate Function ayarı CubeMX tarafından yapıldığı için genelde boş bırakılır.
}

void Gpio_SetAnalogPinMode(uint32_t port, uint16_t pin, uint8_t enable) {
    // Analog pin modu ayarı.
}

void Gpio_IRQ_Enable(void) {
    // BMI160 kesmesi için hangi hattı kullanıyorsan onu aktif et
    // Örn: HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Gpio_EnableInterrupt(uint32_t port, uint16_t pin, uint8_t edge) {
    // Kesme tetikleme kenarı ayarı (Falling/Rising)
}

void Gpio_IRQ_Init(uint32_t port, uint16_t pin, void (*handler)(uint32_t, uint8_t), uint32_t id) {
    // Sürücüden gelen callback fonksiyonunu bağlamak için kullanılır.
}
/* USER CODE END 2 */
