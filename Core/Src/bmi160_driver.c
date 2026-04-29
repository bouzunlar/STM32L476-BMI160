#include "bmi160_driver.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern I2C_HandleTypeDef hi2c1;
static SemaphoreHandle_t xBmi160Mutex = NULL;

static HAL_StatusTypeDef BMI160_IO_Write(uint8_t regAddr, uint8_t data) {
    return HAL_I2C_Mem_Write(&hi2c1, BMI160_ADDR, regAddr, 1, &data, 1, 100);
}

static HAL_StatusTypeDef BMI160_IO_Read(uint8_t regAddr, uint8_t *pData, uint16_t size) {
    return HAL_I2C_Mem_Read(&hi2c1, BMI160_ADDR, regAddr, 1, pData, size, 100);
}

te_Bmi160_ErrorCodes Bmi160_Open(void* vpParam) {
    uint8_t chipID = 0;

    if (xBmi160Mutex == NULL) {
        xBmi160Mutex = xSemaphoreCreateMutex();
    }

    BMI160_IO_Read(BMI160_REG_CHIPID, &chipID, 1);
    if (chipID != 0xD1) {
        return E_BMI160_ERR_WRONG_ID;
    }

    BMI160_IO_Write(BMI160_REG_CMD, BMI160_CMD_ACC_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(50));
    BMI160_IO_Write(BMI160_REG_CMD, BMI160_CMD_GYRO_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(80));

    return E_BMI160_ERR_NONE;
}

te_Bmi160_ErrorCodes Bmi160_Read(void *pvBuffer, const uint32_t xBytes) {

    uint8_t rawBuffer[12];

    ts_Bmi160Data* pData = (ts_Bmi160Data*)pvBuffer;
    if (pvBuffer == NULL) {
        return E_BMI160_ERR_UNKNOWN;
    }

    if (xSemaphoreTake(xBmi160Mutex, portMAX_DELAY) == pdTRUE) {

        if (BMI160_IO_Read(BMI160_REG_DATA_GYRO, rawBuffer, 12) != HAL_OK) {
            xSemaphoreGive(xBmi160Mutex);
            return E_BMI160_ERR_HW_ERROR;
        }
        xSemaphoreGive(xBmi160Mutex);
    } else {
        return E_BMI160_ERR_RESOURCE_BUSY;
    }

    pData->rawGyroX  = (int16_t)((rawBuffer[1] << 8) | rawBuffer[0]);
    pData->rawGyroY  = (int16_t)((rawBuffer[3] << 8) | rawBuffer[2]);
    pData->rawGyroZ  = (int16_t)((rawBuffer[5] << 8) | rawBuffer[4]);

    pData->rawAccelX = (int16_t)((rawBuffer[7] << 8) | rawBuffer[6]);
    pData->rawAccelY = (int16_t)((rawBuffer[9] << 8) | rawBuffer[8]);
    pData->rawAccelZ = (int16_t)((rawBuffer[11] << 8) | rawBuffer[10]);

    return E_BMI160_ERR_NONE;
}
