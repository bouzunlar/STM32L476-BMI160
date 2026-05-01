/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "bmi160_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include "semphr.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

ts_Bmi160_Data g_ImuData;
SemaphoreHandle_t xDataMutex;
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vSensorReadTask(void *argument)
{
    printf("BMI160 Baslatiliyor...\r\n");

    if (Bmi160_Open(NULL) != E_BMI160_ERR_NONE) {
        printf("BMI160 Baslatilamadi!\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
        printf("BMI160 Baslatildi!\r\n");
    }
    for(;;)
    {
    	ts_Bmi160_Data localData;

    	if (Bmi160_Ioctl(E_BMI160_IOCTL_GET_ALL_DATA, &localData)  == E_BMI160_ERR_NONE)
    	{
    	    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    	    g_ImuData = localData;
    	    xSemaphoreGive(xDataMutex);

    	    printf("GYRO -> X: %5d | Y: %5d | Z: %5d  ||  ACCEL -> X: %5d | Y: %5d | Z: %5d\r\n",
    	           localData.rawGyrX,  localData.rawGyrY,  localData.rawGyrZ,
    	           localData.rawAccX, localData.rawAccY, localData.rawAccZ);
    	}
    	else
    	{

    		printf("BMI160 read error!\r\n");

    	}
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void vActivityTask(void *argument)
{
    const int32_t threshold = 20000;

    for(;;)
    {
    	ts_Bmi160_Data localData;

    	xSemaphoreTake(xDataMutex, portMAX_DELAY);
    	localData = g_ImuData;
    	xSemaphoreGive(xDataMutex);

    	if (abs(localData.rawAccX) > threshold ||
    	    abs(localData.rawAccY) > threshold ||
    	    abs(localData.rawAccZ) > threshold)
    	{
    	    printf("\r\n************************************************\r\n");
    	    printf("  [ALARM] SERT HAREKET YAKALANDI! \r\n");
    	    printf("************************************************\r\n\r\n");

    	    vTaskDelay(pdMS_TO_TICKS(1000));
    	}
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  xDataMutex = xSemaphoreCreateMutex();
  if (xDataMutex == NULL) {
      printf("HATA: Data mutex olusturulamadi!\r\n");
  }
    BaseType_t t1 = xTaskCreate(vSensorReadTask, "SensRead", 256, NULL, 2, NULL);
    BaseType_t t2 = xTaskCreate(vActivityTask, "Activity", 256, NULL, 1, NULL);

    printf("FreeRTOS baslatildi...\r\n");

    vTaskStartScheduler();


  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// printf fonksiyonunun verilerini UART2 üzerinden bilgisayara gönderen köprü
int _write(int file, char *ptr, int len) {
    // Eğer senin UART numaran farklıysa (örneğin huart1) burayı ona göre değiştir
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
