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
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "semphr.h"
#include "bmi160_driver.h"
#include "serial_driver.h"
#include "serial_hal.h"

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
#define SENSOR_TASK_STACK_SIZE      512u
#define PROCESSING_TASK_STACK_SIZE  384u
#define DETECTION_TASK_STACK_SIZE   384u
#define UART_TASK_STACK_SIZE        256u

#define SENSOR_TASK_PRIORITY        3u
#define PROCESSING_TASK_PRIORITY    2u
#define DETECTION_TASK_PRIORITY     2u
#define UART_TASK_PRIORITY     		1u

#define SENSOR_QUEUE_LENGTH         5u
#define DETECTION_QUEUE_LENGTH      5u
#define UART_QUEUE_LENGTH           32u
#define WINDOW_SIZE                 40u

#define GYRO_Z_THRESHOLD_RAW        1966
#define ACCEL_XY_VARIANCE_THRESHOLD 2000000L
#define GYRO_Z_SUM_THRESHOLD  (WINDOW_SIZE * GYRO_Z_THRESHOLD_RAW * 7 / 10)

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
static uint8_t ucDetectionQueueStorage[DETECTION_QUEUE_LENGTH * sizeof(ts_Bmi160_Data)];
static uint8_t ucUartTxQueueStorage[UART_QUEUE_LENGTH * sizeof(UartMsg_t)];

serialInterrupt_t serial_irq_context[SERIAL_MAX_INTERRUPT_HANDLERS] = { 0 };

static uint8_t rxByte = 0;

volatile SystemDiagnostics_t g_diag = { 0 };

/* USER CODE END PV */

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

void Serial_RxCallback(void *id)
{
	HAL_UART_Receive_IT(&huart[USER_SERIAL_INDEX], &rxByte, 1);
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

	TickType_t xLastWakeTime = xTaskGetTickCount();

	for (;;)
	{
//		Debug_Print("[TASK] Sensor running\r\n");
		ts_Bmi160_Data localData;

		if (Bmi160_Ioctl(E_BMI160_IOCTL_GET_ALL_DATA, &localData) == E_BMI160_ERR_NONE)
		{

			if (xQueueSend(xSensorQueue, &localData,pdMS_TO_TICKS(5)) != pdTRUE)
			{
				g_diag.sensor_queue_full++;
			}

			static uint32_t dbgCounter = 0;

			if ((dbgCounter++ % 10) == 0)
			{
				char debugMsg[128];

				snprintf(debugMsg, sizeof(debugMsg), "GYRO -> X: %5d | Y: %5d | Z: %5d  ||  ACCEL -> X: %5d | Y: %5d | Z: %5d\r\n", localData.rawGyrX, localData.rawGyrY, localData.rawGyrZ, localData.rawAccX, localData.rawAccY, localData.rawAccZ);

				Debug_Print(debugMsg);
			}

		}
		else
		{

			Debug_Print("BMI160 read error!\r\n");

		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

	}
}

void vDataProcessingTask(void *argument)
{
	Debug_Print("[PROC TASK CREATED]\r\n");
	for (;;)
	{

		ts_Bmi160_Data rawData;

		if (xQueueReceive(xSensorQueue, &rawData, pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
		{

//TODO: data proccessing algorithm need to be implemented.

			if (xQueueSend(xDetectionQueue, &rawData,pdMS_TO_TICKS(5)) != pdTRUE)
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

	/* Sliding window buffer */
	static ts_Bmi160_Data window[WINDOW_SIZE];
	static uint8_t windowIndex = 0;
	static uint8_t windowFull = 0;

	for (;;)
	{
		ts_Bmi160_Data localData;

		if (xQueueReceive(xDetectionQueue, &localData, pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
		{

			window[windowIndex] = localData;
			windowIndex = (uint8_t) ((windowIndex + 1u) % WINDOW_SIZE);
			if (windowIndex == 0u)
			{
				windowFull = 1;
			}

			uint8_t circleFlag = 0;

			if (windowFull)
			{

				int32_t gyroZSum = 0;
				int32_t quarter[4] = { 0, 0, 0, 0 };
				for (uint8_t i = 0; i < WINDOW_SIZE; i++)
				{
					gyroZSum += window[i].rawGyrZ;
					quarter[i / (WINDOW_SIZE / 4)] += window[i].rawGyrZ;
				}

				uint8_t directionConsistent = 1;
				int8_t expectedSign = (gyroZSum > 0) ? 1 : -1;
				for (uint8_t q = 0; q < 4; q++)
				{
					int8_t qSign = (quarter[q] > 0) ? 1 : -1;
					if (qSign != expectedSign)
					{
						directionConsistent = 0;
						break;
					}
				}

				int32_t meanX = 0, meanY = 0;
				for (uint8_t i = 0; i < WINDOW_SIZE; i++)
				{
					meanX += window[i].rawAccX;
					meanY += window[i].rawAccY;
				}
				meanX /= (int32_t) WINDOW_SIZE;
				meanY /= (int32_t) WINDOW_SIZE;

				int64_t varX = 0, varY = 0, covXY = 0;
				uint8_t zcx = 0, zcy = 0;

				for (uint8_t i = 0; i < WINDOW_SIZE; i++)
				{
					int32_t dx = window[i].rawAccX - meanX;
					int32_t dy = window[i].rawAccY - meanY;

					varX += (int64_t) dx * dx;
					varY += (int64_t) dy * dy;
					covXY += (int64_t) dx * dy;

					if (i > 0)
					{
						int32_t prev_dx = window[i - 1].rawAccX - meanX;
						int32_t prev_dy = window[i - 1].rawAccY - meanY;

						if ((prev_dx > 0 && dx <= 0) || (prev_dx < 0 && dx >= 0))
							zcx++;
						if ((prev_dy > 0 && dy <= 0) || (prev_dy < 0 && dy >= 0))
							zcy++;
					}
				}
				varX /= WINDOW_SIZE;
				varY /= WINDOW_SIZE;
				covXY /= WINDOW_SIZE;

				uint8_t notStraightLine = ((covXY * covXY) < (varX * varY / 2));

				uint8_t isFullCircle = (zcx >= 2) && (zcy >= 2);

				if (directionConsistent && notStraightLine && isFullCircle && abs(gyroZSum) > GYRO_Z_SUM_THRESHOLD && varX > ACCEL_XY_VARIANCE_THRESHOLD && varY > ACCEL_XY_VARIANCE_THRESHOLD)
				{
					circleFlag = 1;

					windowFull = 0;
					windowIndex = 0;
					memset(window, 0, sizeof(window));
					xQueueReset(xDetectionQueue);
				}

				if (circleFlag)
				{
					Debug_Print("\r\n************************************************\r\n");
					Debug_Print("[ALARM] CIRCULAR MOTION DETECTED!\r\n");
					Debug_Print("************************************************\r\n");
				}
			}
			else
			{
				g_diag.uart_mutex_timeouts++;
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
			Serial_Write(&txCtx, pkt.len);
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

//	MX_USART2_UART_Init();
	//TODO: serial init. function has to be implemented included below definitions
	uint8_t serialIndex = USER_SERIAL_INDEX;
	Serial_Open(&serialIndex);
	printf("Serial driver started\r\n");
	serialInterrupt_t rxInt;
	rxInt.index = USER_SERIAL_INDEX;
	rxInt.interrupt_handler = Serial_RxCallback;
	rxInt.interrupt_id = USER_SERIAL_INDEX;
	rxInt.isInterruptOccured = 0;
	rxInt.type = 0;
	Serial_Ioctl(E_SERIAL_IOCTL_REGISTER_IRQ_HANDLER, &rxInt);

	Serial_Ioctl(E_SERIAL_IOCTL_ENABLE_INTERRUPT, &rxInt);

	HAL_UART_Receive_IT(&huart[USER_SERIAL_INDEX], &rxByte, 1);

	xSensorQueue = xQueueCreateStatic(SENSOR_QUEUE_LENGTH, sizeof(ts_Bmi160_Data), ucSensorQueueStorage, &xSensorQueueCB);

	xDetectionQueue = xQueueCreateStatic(DETECTION_QUEUE_LENGTH, sizeof(ts_Bmi160_Data), ucDetectionQueueStorage, &xDetectionQueueCB);

	xUartTxQueue = xQueueCreateStatic(8u, sizeof(UartMsg_t), ucUartTxQueueStorage, &xUartTxQueueCB);

	if (xSensorQueue == NULL || xDetectionQueue == NULL || xUartTxQueue == NULL)
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

int _write(int file, char *ptr, int len)
{

//	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	SerialReadWrite_t txCtx;
	txCtx.index = USER_SERIAL_INDEX;
	txCtx.ptr = (uint8_t*) ptr;
	txCtx.length = (uint16_t) len;
	Serial_Write(&txCtx, (uint32_t) len);
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
