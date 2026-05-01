#include "bmi160_lib.h"
#include "bmi160_hal.h"
#include <string.h>
#include "i2c.h"


/* ---- Utility ---------------------------------------------------------- */

void bmi160_delay_ms(unsigned long ms)
{
    HAL_Delay(ms);
}

uint8_t IMU6x_uReadWhoAmI(void)
{
    uint8_t tmp = 0u;
    HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR, BMI160_REG_CHIP_ID, 1, &tmp, 1, 100);
    return tmp;
}

/* Soft reset + power-on acc and gyro */
void IMU6x_vStart(void)
{
    uint8_t cmd;

    cmd = BMI160_CMD_SOFTRESET;
    HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
    bmi160_delay_ms(BMI160_SOFTRESET_DELAY_MS);

    cmd = BMI160_CMD_ACC_NORMAL;
    HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
    bmi160_delay_ms(BMI160_ACC_NORMAL_DELAY_MS);

    cmd = BMI160_CMD_GYR_NORMAL;
    HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, BMI160_REG_CMD, 1, &cmd, 1, 100);
    bmi160_delay_ms(BMI160_GYR_NORMAL_DELAY_MS);
}


