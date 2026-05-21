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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "semphr.h"
#include "bmi160_driver.h"
#include "serial_driver.h"
#include "serial_hal.h"
#include "interrupt.h"
#include "filter.h"
#include "detection.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	char msg[96];
	uint16_t len;
} UartMsg_t;

typedef struct
{
	uint32_t sensor_read_errors;
	uint32_t sensor_queue_full;
	uint32_t detection_queue_full;
	uint32_t upstream_timeouts;
	uint32_t circle_detections;
	uint32_t uart_mutex_timeouts;
} SystemDiagnostics_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_TASK_STACK_SIZE      1024u
#define PROCESSING_TASK_STACK_SIZE  512u
#define DETECTION_TASK_STACK_SIZE   512u
#define UART_TASK_STACK_SIZE        512u

#define SENSOR_TASK_PRIORITY        3u
#define PROCESSING_TASK_PRIORITY    2u
#define DETECTION_TASK_PRIORITY     2u
#define UART_TASK_PRIORITY     		1u

#define SENSOR_QUEUE_LENGTH         5u
#define DETECTION_QUEUE_LENGTH      5u
#define UART_QUEUE_LENGTH           32u

#define QUEUE_RECV_TIMEOUT_MS   	200u

extern UART_HandleTypeDef huart[];

static QueueHandle_t xSensorQueue = NULL;
static QueueHandle_t xDetectionQueue = NULL;
static QueueHandle_t xUartTxQueue = NULL;

static TaskHandle_t xSensorTaskHandle = NULL;
static TaskHandle_t xProcessingTaskHandle = NULL;
static TaskHandle_t xDetectionTaskHandle = NULL;
static TaskHandle_t xUartTxTaskHandle = NULL;

static StaticQueue_t xSensorQueueCB;
static StaticQueue_t xDetectionQueueCB;
static StaticQueue_t xUartTxQueueCB;

static StaticTask_t xSensorTaskTCB;
static StaticTask_t xProcessingTaskTCB;
static StaticTask_t xDetectionTaskTCB;
static StaticTask_t xUartTxTaskTCB;

static SemaphoreHandle_t xBmi160DataReadySem = NULL;
static StaticSemaphore_t xBmi160DataReadySemBuffer;

/* USER CODE END PD */

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static StackType_t xSensorTaskStack[SENSOR_TASK_STACK_SIZE];
static StackType_t xProcessingTaskStack[PROCESSING_TASK_STACK_SIZE];
static StackType_t xDetectionTaskStack[DETECTION_TASK_STACK_SIZE];
static StackType_t xUartTxTaskStack[UART_TASK_STACK_SIZE];

static uint8_t ucSensorQueueStorage[SENSOR_QUEUE_LENGTH * sizeof(ts_Bmi160_Data)];
static uint8_t ucDetectionQueueStorage[DETECTION_QUEUE_LENGTH * sizeof(ts_Processed_Data)];
static uint8_t ucUartTxQueueStorage[UART_QUEUE_LENGTH * sizeof(UartMsg_t)];

volatile SystemDiagnostics_t g_diag = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void Debug_Print(const char *msg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
	static StaticTask_t xIdleTaskTCB;
	static StackType_t xIdleTaskStack[configMINIMAL_STACK_SIZE];
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
	*ppxIdleTaskStackBuffer = xIdleTaskStack;
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
	static StaticTask_t xTimerTaskTCB;
	static StackType_t xTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
	*ppxTimerTaskStackBuffer = xTimerTaskStack;
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vSensorReadTask(void *argument)
{
	Debug_Print("BMI160 is starting...\r\n");
	uint8_t retryCount = 0;
	while (Bmi160_Open(NULL) != E_BMI160_ERR_NONE)
	{
		retryCount++;
		char msg[64];

		snprintf(msg, sizeof(msg), "BMI160 failed to start! Attempt: %d\r\n", retryCount);
		Debug_Print(msg);
		if (retryCount >= 3u)
		{
			Debug_Print("BMI160 Failed to start after 3 attempts!\r\n");
			vTaskDelay(pdMS_TO_TICKS(100));
			NVIC_SystemReset();
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
	Debug_Print("BMI160 Started!\r\n");

	if (Bmi160_Ioctl(E_BMI160_IOCTL_REGISTER_INTERRUPT_HANDLER, (void*) xBmi160DataReadySem) != E_BMI160_ERR_NONE)
	{
		Debug_Print("BMI160: ISR register FAILED\r\n");
	}

	if (Bmi160_Ioctl(E_BMI160_IOCTL_ENABLE_INTERRUPT, NULL) != E_BMI160_ERR_NONE)
	{
		Debug_Print("BMI160: interrupt enable FAILED\r\n");
	}

	for (;;)
	{
		if (xSemaphoreTake(xBmi160DataReadySem, pdMS_TO_TICKS(100)) != pdTRUE)
		{
			g_diag.sensor_read_errors++;
			Debug_Print("BMI160: DRDY timeout\r\n");
			continue;
		}

		ts_Bmi160_Data localData;

		if (Bmi160_Ioctl(E_BMI160_IOCTL_GET_ALL_DATA, &localData) == E_BMI160_ERR_NONE)
		{
			if (xQueueSend(xSensorQueue, &localData,pdMS_TO_TICKS(5)) != pdTRUE)
			{
				g_diag.sensor_queue_full++;
			}

			static uint32_t dbgCounter = 0;

			if ((dbgCounter++ % 20) == 0)
			{
				char debugMsg[128];

				snprintf(debugMsg, sizeof(debugMsg),
					"GYRO[dps] X:%7.2f Y:%7.2f Z:%7.2f | ACC[g] X:%6.3f Y:%6.3f Z:%6.3f\r\n",
					localData.scaledGyrX, localData.scaledGyrY, localData.scaledGyrZ,
					localData.scaledAccX, localData.scaledAccY, localData.scaledAccZ);

				Debug_Print(debugMsg);
			}
		}
		else
		{
			Debug_Print("BMI160 read error!\r\n");
		}
	}
}

void vDataProcessingTask(void *argument)
{
	Debug_Print("[PROC TASK CREATED]\r\n");

	Filter_Init();

	for (;;)
	{
		ts_Bmi160_Data rawData;

		if (xQueueReceive(xSensorQueue, &rawData, pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
		{
			ts_Processed_Data out;
			Filter_Apply(&rawData, &out);

			if (xQueueSend(xDetectionQueue, &out, pdMS_TO_TICKS(5)) != pdTRUE)
			{
				g_diag.detection_queue_full++;
			}
		}
		else
		{
			g_diag.upstream_timeouts++;
		}
	}
}

void vCircleDetectionTask(void *argument)
{
	Debug_Print("[DETECT TASK CREATED]\r\n");

	Detection_Init();

	for (;;)
	{
		ts_Processed_Data sample;

		if (xQueueReceive(xDetectionQueue, &sample, pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
		{
			ts_Detection_Result result = Detection_Process(&sample);

			if (result.detected)
			{
				g_diag.circle_detections++;

				char msg[96];
				snprintf(msg, sizeof(msg),
					"[ALARM] CIRCULAR MOTION DETECTED! %s | meanGyrZ=%.1f dps varX=%.3f varY=%.3f\r\n",
					(result.direction > 0) ? "CW" : "CCW",
					result.meanGyrZ, result.varX, result.varY);

				Debug_Print("\r\n************************************************\r\n");
				Debug_Print(msg);
				Debug_Print("************************************************\r\n");

					xQueueReset(xDetectionQueue);
				}
			//TODO: UART packet format implementation has to be done.

			//TODO: Sending uart data.

		}
		else
		{
			g_diag.upstream_timeouts++;
		}
	}
}

void vUartTxTask(void *argument)
{
	UartMsg_t pkt;
	for (;;)
	{
		if (xQueueReceive(xUartTxQueue, &pkt, portMAX_DELAY) == pdTRUE)
		{
			SerialReadWrite_t txCtx;
			txCtx.index = USER_SERIAL_INDEX;
			txCtx.ptr = (uint8_t*) pkt.msg;
			txCtx.length = pkt.len;
			if (Serial_Write_DMA(&txCtx, pkt.len) == E_SERIAL_ERR_NONE)
			{
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

			}
			else
			{
				//TODO: error handling
			}

		}
	}
}

void Debug_Print(const char *msg)
{
	if (msg == NULL || xUartTxQueue == NULL)
		return;
	UartMsg_t pkt;
	int written = snprintf(pkt.msg, sizeof(pkt.msg), "%s", msg);

	if (written < 0)
	{
		return;
	}

	if (written >= sizeof(pkt.msg))
	{
		pkt.len = sizeof(pkt.msg) - 1u;
	}
	else
	{
		pkt.len = (uint16_t) written;
	}
	xQueueSend(xUartTxQueue, &pkt, pdMS_TO_TICKS(5));
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();

	Interrupt_Init();

	uint8_t serialIndex = USER_SERIAL_INDEX;
	Serial_Open(&serialIndex);
	Serial_Ioctl(E_SERIAL_IOCTL_ENABLE_DMA_TX, &serialIndex); //DMA started

	printf("Serial driver started\r\n");

	xSensorQueue = xQueueCreateStatic(SENSOR_QUEUE_LENGTH, sizeof(ts_Bmi160_Data), ucSensorQueueStorage, &xSensorQueueCB);

	xDetectionQueue = xQueueCreateStatic(DETECTION_QUEUE_LENGTH, sizeof(ts_Processed_Data), ucDetectionQueueStorage, &xDetectionQueueCB);

	xUartTxQueue = xQueueCreateStatic(UART_QUEUE_LENGTH, sizeof(UartMsg_t), ucUartTxQueueStorage, &xUartTxQueueCB);

	xBmi160DataReadySem = xSemaphoreCreateBinaryStatic(&xBmi160DataReadySemBuffer);

	if (xSensorQueue == NULL || xDetectionQueue == NULL || xUartTxQueue == NULL || xBmi160DataReadySem == NULL)
	{
		printf("FreeRTOS object creation failed!\r\n");
		Error_Handler();
	}

	xSensorTaskHandle = xTaskCreateStatic(vSensorReadTask, "SensRead", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, xSensorTaskStack, &xSensorTaskTCB);

	xProcessingTaskHandle = xTaskCreateStatic(vDataProcessingTask, "DataProc", PROCESSING_TASK_STACK_SIZE, NULL, PROCESSING_TASK_PRIORITY, xProcessingTaskStack, &xProcessingTaskTCB);

	xDetectionTaskHandle = xTaskCreateStatic(vCircleDetectionTask, "CircDet", DETECTION_TASK_STACK_SIZE, NULL, DETECTION_TASK_PRIORITY, xDetectionTaskStack, &xDetectionTaskTCB);

	xUartTxTaskHandle = xTaskCreateStatic(vUartTxTask, "UartTx", UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY, xUartTxTaskStack, &xUartTxTaskTCB);

	if (xSensorTaskHandle == NULL || xProcessingTaskHandle == NULL || xDetectionTaskHandle == NULL || xUartTxTaskHandle == NULL)
	{
		printf("Task creation failed!\r\n");
		Error_Handler();
	}

	printf("FreeRTOS is starting...\r\n");

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		if (xUartTxTaskHandle != NULL)
		{
			vTaskNotifyGiveFromISR(xUartTxTaskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}

int _write(int file, char *ptr, int len)
{

//	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	 if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	    {
	        if (xUartTxQueue != NULL)
	        {
	            UartMsg_t pkt;
	            int written = snprintf(pkt.msg, sizeof(pkt.msg), "%.*s", len, ptr);
	            pkt.len = (uint16_t)((written > 0) ? written : 0);
	            xQueueSend(xUartTxQueue, &pkt, pdMS_TO_TICKS(5));
	        }
	        return len;
	    }

	    SerialReadWrite_t txCtx;
	    txCtx.index  = USER_SERIAL_INDEX;
	    txCtx.ptr    = (uint8_t*) ptr;
	    txCtx.length = (uint16_t) len;
	    Serial_Write(&txCtx, (uint32_t) len);
	    return len;
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    // Breakpoint here 
    taskDISABLE_INTERRUPTS();
    for(;;);
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
	taskDISABLE_INTERRUPTS(); //stop the freertos scheduler
	NVIC_SystemReset();

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
