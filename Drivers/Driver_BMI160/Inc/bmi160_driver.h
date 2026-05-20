#ifndef BMI160_DRIVER_H_
#define BMI160_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bmi160_hal.h"
#include "main.h"



#define BMI160_DRIVER_SW_VERSION    (1.0)

typedef void (*bmi160_irq_handler)(uint32_t id, uint8_t event);

typedef struct
{
    uint8_t            isFallingOrRising;
    uint32_t           interrupt_id;
    bmi160_irq_handler interrupt_handler;
} ts_Bmi160_Interrupt;

typedef enum
{
    E_BMI160_ERR_NONE,
    E_BMI160_ERR_WRONG_ID,
    E_BMI160_ERR_HW_ERROR,
    E_BMI160_ERR_WRONG_IOCTL_CMD,
    E_BMI160_ERR_UNKNOWN
} te_Bmi160_ErrorCodes;


/* ---- Sensor data struct ------------------------------------------------ */
typedef struct
{
    int16_t rawAccX,  rawAccY,  rawAccZ;
    int16_t rawGyrX,  rawGyrY,  rawGyrZ;
    float   scaledAccX, scaledAccY, scaledAccZ;  /* g     */
    float   scaledGyrX, scaledGyrY, scaledGyrZ;  /* deg/s */
    float   temperature;                          /* °C    */
} ts_Bmi160_Data;


typedef enum
{
    E_BMI160_IOCTL_GET_VERSION,

    E_BMI160_IOCTL_GET_WHO_AM_I,

    E_BMI160_IOCTL_GET_ACC_RAW_X,
    E_BMI160_IOCTL_GET_ACC_RAW_Y,
    E_BMI160_IOCTL_GET_ACC_RAW_Z,
    E_BMI160_IOCTL_GET_ACC_SCALED_X,
    E_BMI160_IOCTL_GET_ACC_SCALED_Y,
    E_BMI160_IOCTL_GET_ACC_SCALED_Z,

    E_BMI160_IOCTL_GET_GYRO_RAW_X,
    E_BMI160_IOCTL_GET_GYRO_RAW_Y,
    E_BMI160_IOCTL_GET_GYRO_RAW_Z,
    E_BMI160_IOCTL_GET_GYRO_SCALED_X,
    E_BMI160_IOCTL_GET_GYRO_SCALED_Y,
    E_BMI160_IOCTL_GET_GYRO_SCALED_Z,

    E_BMI160_IOCTL_GET_TEMPERATURE,

    E_BMI160_IOCTL_GET_ALL_DATA,

    E_BMI160_IOCTL_SET_SAMPLE_RATE,

    E_BMI160_IOCTL_ENABLE_INTERRUPT,
    E_BMI160_IOCTL_REGISTER_INTERRUPT_HANDLER,

}te_Bmi160_IoctlCommands;

te_Bmi160_ErrorCodes Bmi160_Open (void *vpParam);
te_Bmi160_ErrorCodes Bmi160_Ioctl(te_Bmi160_IoctlCommands eCommand, void *vpParam);
te_Bmi160_ErrorCodes Bmi160_Write(const void *pvBuffer, uint32_t xBytes);
te_Bmi160_ErrorCodes Bmi160_Read (const void *pvBuffer, uint32_t xBytes);
te_Bmi160_ErrorCodes Bmi160_Close(void *vpParam);

int8_t BMI160_IMU_TEST(uint32_t timeout_ms);

void    IMU6x_vHWInit    (void);
uint8_t IMU6x_uReadWhoAmI(void);
void    IMU6x_vStart     (void);   /* soft reset + acc/gyr normal mode */

/* Utility */
void bmi160_delay_ms(unsigned long ms);

#ifdef __cplusplus
}
#endif

#endif /* BMI160_DRIVER_H_ */
