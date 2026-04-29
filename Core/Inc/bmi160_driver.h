#ifndef BMI160_DRIVER_H_
#define BMI160_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bmi160_hal.h"
#include "main.h"

typedef enum
{
    E_BMI160_ERR_NONE = 0,
    E_BMI160_ERR_WRONG_ID,
    E_BMI160_ERR_HW_ERROR,
    E_BMI160_ERR_RESOURCE_BUSY,
    E_BMI160_ERR_UNKNOWN
} te_Bmi160_ErrorCodes;

typedef struct
{
    int16_t rawGyroX;
    int16_t rawGyroY;
    int16_t rawGyroZ;

    int16_t rawAccelX;
    int16_t rawAccelY;
    int16_t rawAccelZ;
} ts_Bmi160Data;

te_Bmi160_ErrorCodes Bmi160_Open(void* vpParam);
te_Bmi160_ErrorCodes Bmi160_Read(void *pvBuffer, const uint32_t xBytes);

#ifdef __cplusplus
}
#endif

#endif /* BMI160_DRIVER_H_ */
