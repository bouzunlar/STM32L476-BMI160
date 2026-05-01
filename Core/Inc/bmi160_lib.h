/*
 * bmi160_lib.h
 *
 *  Created on: May 1, 2026
 *      Author: buketozturk
 */

#ifndef INC_BMI160_LIB_H_
#define INC_BMI160_LIB_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#ifdef __FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#define i2cENTER_CRITICAL()   taskENTER_CRITICAL()
#define i2cEXIT_CRITICAL()    taskEXIT_CRITICAL()
#else
#define i2cENTER_CRITICAL()
#define i2cEXIT_CRITICAL()
#endif


void    IMU6x_vHWInit    (void);
uint8_t IMU6x_uReadWhoAmI(void);
void    IMU6x_vStart     (void);   /* soft reset + acc/gyr normal mode */

/* Utility */
void bmi160_delay_ms(unsigned long ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_BMI160_LIB_H_ */
