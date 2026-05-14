#include "../../Drivers/Driver_BMI160/Inc/bmi160_driver.h"
#include "../../Drivers/Driver_BMI160/Inc/bmi160_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

static HAL_StatusTypeDef BMI160_IO_Write(uint8_t regAddr, uint8_t data)
{
	return HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, regAddr, 1, &data, 1, 100);
}

static HAL_StatusTypeDef BMI160_IO_Read(uint8_t regAddr, uint8_t *pData, uint16_t size)
{
	return HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR, regAddr, 1, pData, size, 100);
}

/* Raw data buffers short gyro[3], accel[3]  globals */
static short accel[3];
static short gyro[3];
static short temperature_raw;

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

static te_Bmi160_ErrorCodes BMI160_vInit(void)
{

	IMU6x_vStart(); /* soft reset + acc/gyr normal mode */

	/* Configure acc: 100 Hz, Normal BWP, ±4G */
	uint8_t reg = (uint8_t) ((BMI160_ACC_BWP_NORMAL << 4u) | BMI160_ACC_ODR_100HZ);
	BMI160_IO_Write(BMI160_REG_ACC_CONF, reg);
	BMI160_IO_Write(BMI160_REG_ACC_RANGE, BMI160_ACC_RANGE_4G);

	/* Configure gyro: 100 Hz, Normal BWP, ±500 DPS */
	reg = (uint8_t) ((BMI160_GYR_BWP_NORMAL << 4u) | BMI160_GYR_ODR_100HZ);
	BMI160_IO_Write(BMI160_REG_GYR_CONF, reg);
	BMI160_IO_Write(BMI160_REG_GYR_RANGE, BMI160_GYR_RANGE_500DPS);

	printf("BMI160: Init OK, ID=0xD1, ODR=100Hz, Acc=4G, Gyr=500dps\r\n");
	return E_BMI160_ERR_NONE;
}

te_Bmi160_ErrorCodes Bmi160_Open(void *vpParam)
{

	if (IMU6x_uReadWhoAmI() != BMI160_CHIP_ID)
	{
		printf("BMI160: Wrong ID!!!\r\n");
		return E_BMI160_ERR_WRONG_ID;
	}

	return BMI160_vInit();

}

static void BMI160_ReadSensors(void)
{
	uint8_t buf[12];

	/* Gyro: 0x0C-0x11 */
	BMI160_IO_Read(BMI160_REG_DATA_GYR_X_L, buf, 6u);
	gyro[0] = (short) ((unsigned short) buf[1] << 8u | buf[0]);
	gyro[1] = (short) ((unsigned short) buf[3] << 8u | buf[2]);
	gyro[2] = (short) ((unsigned short) buf[5] << 8u | buf[4]);

	/* Acc: 0x12-0x17 */
	BMI160_IO_Read(BMI160_REG_DATA_ACC_X_L, buf, 6u);
	accel[0] = (short) ((unsigned short) buf[1] << 8u | buf[0]);
	accel[1] = (short) ((unsigned short) buf[3] << 8u | buf[2]);
	accel[2] = (short) ((unsigned short) buf[5] << 8u | buf[4]);

	/* Temperature: 2 bytes */
	BMI160_IO_Read(BMI160_REG_TEMPERATURE_L, buf, 2u);
	temperature_raw = (short) ((unsigned short) buf[1] << 8u | buf[0]);
}

te_Bmi160_ErrorCodes Bmi160_Ioctl(te_Bmi160_IoctlCommands eCommand, void *vpParam)
{

	ts_Bmi160_Data *pData;
//	ts_Bmi160_Interrupt *pInt;

	BMI160_ReadSensors();

	switch (eCommand)
	{
	case E_BMI160_IOCTL_GET_VERSION:
		*(float*) vpParam = (float) BMI160_DRIVER_SW_VERSION;
		break;

	case E_BMI160_IOCTL_GET_WHO_AM_I:
		*(uint8_t*) vpParam = IMU6x_uReadWhoAmI();
		break;

		/* Accelerometer raw */
	case E_BMI160_IOCTL_GET_ACC_RAW_X:
		*(int16_t*) vpParam = accel[0];
		break;
	case E_BMI160_IOCTL_GET_ACC_RAW_Y:
		*(int16_t*) vpParam = accel[1];
		break;
	case E_BMI160_IOCTL_GET_ACC_RAW_Z:
		*(int16_t*) vpParam = accel[2];
		break;

		/* Accelerometer scaled */
	case E_BMI160_IOCTL_GET_ACC_SCALED_X:
		*(float*) vpParam = (float) accel[0] * BMI160_ACC_SCALE_4G;
		break;
	case E_BMI160_IOCTL_GET_ACC_SCALED_Y:
		*(float*) vpParam = (float) accel[1] * BMI160_ACC_SCALE_4G;
		break;
	case E_BMI160_IOCTL_GET_ACC_SCALED_Z:
		*(float*) vpParam = (float) accel[2] * BMI160_ACC_SCALE_4G;
		break;

		/* Gyroscope raw */
	case E_BMI160_IOCTL_GET_GYRO_RAW_X:
		*(int16_t*) vpParam = gyro[0];
		break;
	case E_BMI160_IOCTL_GET_GYRO_RAW_Y:
		*(int16_t*) vpParam = gyro[1];
		break;
	case E_BMI160_IOCTL_GET_GYRO_RAW_Z:
		*(int16_t*) vpParam = gyro[2];
		break;

		/* Gyroscope scaled */
	case E_BMI160_IOCTL_GET_GYRO_SCALED_X:
		*(float*) vpParam = (float) gyro[0] * BMI160_GYR_SCALE_500DPS;
		break;
	case E_BMI160_IOCTL_GET_GYRO_SCALED_Y:
		*(float*) vpParam = (float) gyro[1] * BMI160_GYR_SCALE_500DPS;
		break;
	case E_BMI160_IOCTL_GET_GYRO_SCALED_Z:
		*(float*) vpParam = (float) gyro[2] * BMI160_GYR_SCALE_500DPS;
		break;

		/* Temperature */
	case E_BMI160_IOCTL_GET_TEMPERATURE:
		*(float*) vpParam = BMI160_TEMP_OFFSET + (float) temperature_raw * BMI160_TEMP_SCALE;
		break;

		/* All data in one shot */
	case E_BMI160_IOCTL_GET_ALL_DATA:
		pData = (ts_Bmi160_Data*) vpParam;
		pData->rawAccX = accel[0];
		pData->rawAccY = accel[1];
		pData->rawAccZ = accel[2];
		pData->rawGyrX = gyro[0];
		pData->rawGyrY = gyro[1];
		pData->rawGyrZ = gyro[2];
		pData->scaledAccX = (float) accel[0] * BMI160_ACC_SCALE_4G;
		pData->scaledAccY = (float) accel[1] * BMI160_ACC_SCALE_4G;
		pData->scaledAccZ = (float) accel[2] * BMI160_ACC_SCALE_4G;
		pData->scaledGyrX = (float) gyro[0] * BMI160_GYR_SCALE_500DPS;
		pData->scaledGyrY = (float) gyro[1] * BMI160_GYR_SCALE_500DPS;
		pData->scaledGyrZ = (float) gyro[2] * BMI160_GYR_SCALE_500DPS;
		pData->temperature = BMI160_TEMP_OFFSET + (float) temperature_raw * BMI160_TEMP_SCALE;
		break;

		/* Interrupt */
//		case E_BMI160_IOCTL_ENABLE_INTERRUPT:
//			Gpio_IRQ_Enable();
//			break;
//
//		case E_BMI160_IOCTL_REGISTER_INTERRUPT_HANDLER:
//			pInt = (ts_Bmi160_Interrupt *)vpParam;
//			Gpio_EnableInterrupt(BMI160_SENS_INT_PORTNUM,
//								 BMI160_SENS_INT_PINNUM,
//								 pInt->isFallingOrRising);
//			Gpio_IRQ_Init(BMI160_SENS_INT_PORTNUM,
//						  BMI160_SENS_INT_PINNUM,
//						  pInt->interrupt_handler,
//						  pInt->interrupt_id);
//			break;
	default:
		return E_BMI160_ERR_WRONG_IOCTL_CMD;
	}

	return E_BMI160_ERR_NONE;
}

int8_t BMI160_IMU_TEST(uint32_t timeout_ms)
{
	uint32_t timeCnt = 0u;
	ts_Bmi160_Data xData;

	if (Bmi160_Open(NULL) != E_BMI160_ERR_NONE)
	{
		printf("BMI160: Init FAILED\r\n");
		return -1;
	}

	while (timeCnt < timeout_ms)
	{
		Bmi160_Ioctl(E_BMI160_IOCTL_GET_ALL_DATA, &xData);

		printf("AccX:%f\tAccY:%f\tAccZ:%f\t[g]\r\n", xData.scaledAccX, xData.scaledAccY, xData.scaledAccZ);
		printf("GyrX:%f\tGyrY:%f\tGyrZ:%f\t[dps]\r\n", xData.scaledGyrX, xData.scaledGyrY, xData.scaledGyrZ);
		printf("Temp:%.1f C\r\n\n", xData.temperature);

		bmi160_delay_ms(100u);
		timeCnt += 100u;
	}

	if (xData.rawAccX == 0 && xData.rawAccY == 0 && xData.rawAccZ == 0)
	{
		printf("BMI160: TEST FAILED\r\n");
		return -1;
	}

	printf("BMI160: TEST PASSED\r\n");
	return 0;
}
