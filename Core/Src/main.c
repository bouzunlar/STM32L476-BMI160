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
#include "fusion.h"
#include "detection.h"
#include "telemetry.h"
#include "uart_tx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
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
#define PROCESSING_TASK_STACK_SIZE  1024u
#define DETECTION_TASK_STACK_SIZE   512u
#define DIAG_TASK_STACK_SIZE        384u

#define SENSOR_TASK_PRIORITY        3u
#define PROCESSING_TASK_PRIORITY    2u
#define DETECTION_TASK_PRIORITY     2u
#define DIAG_TASK_PRIORITY          1u

#define SENSOR_QUEUE_LENGTH         5u
#define DETECTION_QUEUE_LENGTH      5u

#define QUEUE_RECV_TIMEOUT_MS   	200u


#define DIAG_PERIOD_MS              (5000u)
#define IWDG_TIMEOUT_MS_APPROX      (8000u)


#ifndef DIAG_TEST
#define DIAG_TEST                (0) // 0 = clean demo, 1 = report screenshots

#endif

static QueueHandle_t xSensorQueue = NULL;
static QueueHandle_t xDetectionQueue = NULL;

static TaskHandle_t xSensorTaskHandle = NULL;
static TaskHandle_t xProcessingTaskHandle = NULL;
static TaskHandle_t xDetectionTaskHandle = NULL;
static TaskHandle_t xDiagTaskHandle = NULL;

static StaticQueue_t xSensorQueueCB;
static StaticQueue_t xDetectionQueueCB;

static StaticTask_t xSensorTaskTCB;
static StaticTask_t xProcessingTaskTCB;
static StaticTask_t xDetectionTaskTCB;
static StaticTask_t xDiagTaskTCB;

/* Per-task heartbeats; the diag task refreshes the IWDG only when all of
 * them advance, proving every task is alive. */
static volatile uint32_t s_hbSensor;
static volatile uint32_t s_hbProc;
static volatile uint32_t s_hbDet;

/* Inter-sample jitter monitor (ticks): validates Mahony's fixed 10 ms dt by
 * measuring the gap between DRDY samples. */
static volatile uint32_t s_dtMinTicks = 0xFFFFFFFFu;
static volatile uint32_t s_dtMaxTicks = 0u;
static volatile uint32_t s_dtSumTicks = 0u;
static volatile uint32_t s_dtCount    = 0u;

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
static StackType_t xDiagTaskStack[DIAG_TASK_STACK_SIZE];

static uint8_t ucSensorQueueStorage[SENSOR_QUEUE_LENGTH * sizeof(ts_Bmi160_Data)];
static uint8_t ucDetectionQueueStorage[DETECTION_QUEUE_LENGTH * sizeof(ts_Processed_Data)];

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

	TickType_t lastSampleTick = 0u;

	for (;;)
	{
		s_hbSensor++;   /* heartbeat for watchdog */

		if (xSemaphoreTake(xBmi160DataReadySem, pdMS_TO_TICKS(100)) != pdTRUE)
		{
			g_diag.sensor_read_errors++;
			Debug_Print("BMI160: DRDY timeout\r\n");
			lastSampleTick = 0u;   /* break jitter window after a stall */
			continue;
		}

		/* --- Inter-sample jitter measurement ---*/
		TickType_t now = xTaskGetTickCount();
		if (lastSampleTick != 0u)
		{
			uint32_t dt = (uint32_t)(now - lastSampleTick);
			s_dtSumTicks += dt;
			s_dtCount++;
			if (dt < s_dtMinTicks) s_dtMinTicks = dt;
			if (dt > s_dtMaxTicks) s_dtMaxTicks = dt;
		}
		lastSampleTick = now;

		ts_Bmi160_Data localData;

		if (Bmi160_Ioctl(E_BMI160_IOCTL_GET_ALL_DATA, &localData) == E_BMI160_ERR_NONE)
		{
			if (xQueueSend(xSensorQueue, &localData, pdMS_TO_TICKS(5)) != pdTRUE)
			{
				g_diag.sensor_queue_full++;
			}

#if DIAG_TEST
			/* ASCII debug print — readable in terminal. Only compiled in when
			 * DIAG_TEST=1. For the binary-telemetry demo this stays out so
			 * GYRO/ACC text does not collide with the framed payload. */
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
#endif
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

	Fusion_Init();

	for (;;)
	{
		s_hbProc++;   /* heartbeat for watchdog */

		ts_Bmi160_Data rawData;

		if (xQueueReceive(xSensorQueue, &rawData, pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
		{
			ts_Processed_Data out;
			Fusion_Apply(&rawData, &out);   /* Mahony PI fusion + cumulative yaw */

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
	Telemetry_Init();

	for (;;)
	{
		s_hbDet++;   /* heartbeat for watchdog */

		ts_Processed_Data sample;

		if (xQueueReceive(xDetectionQueue, &sample, pdMS_TO_TICKS(QUEUE_RECV_TIMEOUT_MS)) == pdTRUE)
		{
			ts_Detection_Result result = Detection_Process(&sample);

			uint16_t flags = 0u;
			if (result.detected)
			{
				flags |= TELEMETRY_FLAG_DETECTION;
			}

			/* DIAG_TEST=1 -> ASCII debug only; DIAG_TEST=0 -> binary frame
			 * (the two would corrupt each other on the same UART). */
#if DIAG_TEST
			(void) flags;
			/* Detection-gate trace (~4 Hz) for serial-port tuning. */
			static uint32_t detDbg = 0u;
			if ((detDbg++ % 25u) == 0u)
			{
				char dmsg[160];
				snprintf(dmsg, sizeof(dmsg),
					"[DET] yaw=%6.1f varX=%.3f varY=%.3f ratio=%4.2f q=%u/4 "
					"arc=%+5.0f cum=%+6.0f cand=%u det=%u\r\n",
					result.meanYawRateDps, result.varX, result.varY,
					result.varRatio, result.matchingQuarters,
					result.deltaPsiDeg, result.cumArcDeg,
					result.windowCandidate, result.detected);
				Debug_Print(dmsg);
			}
#else
			Telemetry_SendSensorFrame(&sample, flags);
#endif

			if (result.detected)
			{
				g_diag.circle_detections++;

#if DIAG_TEST
				/* ASCII alarm banner — only in test mode. In demo mode the
				 * detection signal still rides on the binary telemetry frame
				 * (FLAG_DETECTION bit) for the PC viewer to highlight. */
				char msg[128];
				snprintf(msg, sizeof(msg),
					"[ALARM] CIRCULAR MOTION DETECTED! %s | yaw=%.1f dps arc=%.0f deg varX=%.3f varY=%.3f\r\n",
					(result.direction > 0) ? "CW" : "CCW",
					result.meanYawRateDps, result.deltaPsiDeg, result.varX, result.varY);

				Debug_Print("\r\n************************************************\r\n");
				Debug_Print(msg);
				Debug_Print("************************************************\r\n");
#endif

				xQueueReset(xDetectionQueue);
			}
		}
		else
		{
			g_diag.upstream_timeouts++;
		}
	}
}

void Debug_Print(const char *msg)
{
	if (msg == NULL)
	{
		return;
	}

	uint8_t buf[UART_TX_MSG_MAX];
	int written = snprintf((char*) buf, sizeof(buf), "%s", msg);

	if (written <= 0)
	{
		return;
	}

	uint16_t len = (written >= (int) sizeof(buf)) ? (sizeof(buf) - 1u) : (uint16_t) written;
	UartTx_SendBytes(buf, len);
}

/* --------------------------------------------------------------------------
 *  Independent Watchdog (IWDG) — bare-metal driver.
 *
 *  IWDG is enabled via four key-register writes; no HAL module needed.
 *  Clock is the LSI (~32 kHz). With prescaler /64 the counter ticks at
 *  ~500 Hz; a reload value of 4000 gives roughly 8 seconds before the MCU
 *  resets. The diag task refreshes the watchdog every DIAG_PERIOD_MS but only
 *  if every monitored task heartbeat has advanced since the last sample.
 *
 *  Key register write protocol (RM0351 §32.4.1):
 *    KR = 0x5555 → unlock PR / RLR for writing
 *    PR = 4      → prescaler divider /64 (0=/4, 1=/8, 2=/16, 3=/32, 4=/64,
 *                  5=/128, 6=/256)
 *    RLR = N     → reload value (12-bit, max 0x0FFF)
 *    KR = 0xAAAA → reload the counter from RLR (also used to refresh)
 *    KR = 0xCCCC → start the watchdog (only needed once)
 * -------------------------------------------------------------------------- */
#define IWDG_KEY_RELOAD     (0x0000AAAAu)
#define IWDG_KEY_ENABLE     (0x0000CCCCu)
#define IWDG_KEY_WRITE_ACC  (0x00005555u)

static void IWDG_Init(void)
{
	IWDG->KR  = IWDG_KEY_WRITE_ACC;   /* unlock PR/RLR */
	IWDG->PR  = 4u;                   /* /64 prescaler -> ~500 Hz tick */
	IWDG->RLR = 4000u;                /* reload -> ~8 s timeout */
	IWDG->KR  = IWDG_KEY_RELOAD;      /* preload counter from RLR */
	IWDG->KR  = IWDG_KEY_ENABLE;      /* start the watchdog */
}

static inline void IWDG_Refresh(void)
{
	IWDG->KR = IWDG_KEY_RELOAD;
}

/* --------------------------------------------------------------------------
 *  Diagnostics task: periodically reports stack high-water marks, queue
 *  occupancy and accumulated drop counters; kicks the watchdog when every
 *  monitored task heartbeat has moved.
 * -------------------------------------------------------------------------- */
void vDiagTask(void *argument)
{
	(void) argument;
#if DIAG_TEST
	Debug_Print("[DIAG TASK CREATED]\r\n");
#endif

	uint32_t prevHbSensor = 0u;
	uint32_t prevHbProc   = 0u;
	uint32_t prevHbDet    = 0u;

	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(DIAG_PERIOD_MS));

		uint32_t hbSens = s_hbSensor;
		uint32_t hbProc = s_hbProc;
		uint32_t hbDet  = s_hbDet;

		uint8_t allAlive = (hbSens != prevHbSensor) &&
		                    (hbProc != prevHbProc)   &&
		                    (hbDet  != prevHbDet);

		prevHbSensor = hbSens;
		prevHbProc   = hbProc;
		prevHbDet    = hbDet;

#if DIAG_TEST
		/* Verbose diagnostics — compile-time gated. Used for the report
		 * (stack high-water mark, queue occupancy, drop counters) and removed
		 * from the demo build for a clean console. */
		UBaseType_t hwmSens = uxTaskGetStackHighWaterMark(xSensorTaskHandle);
		UBaseType_t hwmProc = uxTaskGetStackHighWaterMark(xProcessingTaskHandle);
		UBaseType_t hwmDet  = uxTaskGetStackHighWaterMark(xDetectionTaskHandle);
		UBaseType_t hwmDiag = uxTaskGetStackHighWaterMark(NULL);

		UBaseType_t qSens = uxQueueMessagesWaiting(xSensorQueue);
		UBaseType_t qDet  = uxQueueMessagesWaiting(xDetectionQueue);

		/* Split into two prints so each line fits inside UART_TX_MSG_MAX (96). */
		char msg[96];
		snprintf(msg, sizeof(msg),
			"\r\n[DIAG] StackFree Sens=%lu Proc=%lu Det=%lu Diag=%lu | Q s=%lu/%u d=%lu/%u\r\n",
			(unsigned long) hwmSens, (unsigned long) hwmProc,
			(unsigned long) hwmDet,  (unsigned long) hwmDiag,
			(unsigned long) qSens,   (unsigned) SENSOR_QUEUE_LENGTH,
			(unsigned long) qDet,    (unsigned) DETECTION_QUEUE_LENGTH);
		Debug_Print(msg);

		snprintf(msg, sizeof(msg),
			"[DIAG] Drops sQ=%lu dQ=%lu | sRead=%lu upTO=%lu | Det=%lu | WDog=%s\r\n",
			(unsigned long) g_diag.sensor_queue_full,
			(unsigned long) g_diag.detection_queue_full,
			(unsigned long) g_diag.sensor_read_errors,
			(unsigned long) g_diag.upstream_timeouts,
			(unsigned long) g_diag.circle_detections,
			allAlive ? "KICK" : "HOLD");
		Debug_Print(msg);

		/* Inter-sample jitter snapshot: prove Mahony's fixed-dt assumption.
		 * Atomic read-and-reset; brief interrupt mask keeps min/max coherent. */
		taskENTER_CRITICAL();
		uint32_t dtMin = (s_dtMinTicks == 0xFFFFFFFFu) ? 0u : s_dtMinTicks;
		uint32_t dtMax = s_dtMaxTicks;
		uint32_t dtSum = s_dtSumTicks;
		uint32_t dtN   = s_dtCount;
		s_dtMinTicks = 0xFFFFFFFFu;
		s_dtMaxTicks = 0u;
		s_dtSumTicks = 0u;
		s_dtCount    = 0u;
		taskEXIT_CRITICAL();

		uint32_t dtAvgX10 = (dtN > 0u) ? (uint32_t)((dtSum * 10u) / dtN) : 0u;
		snprintf(msg, sizeof(msg),
			"[DIAG] dt ms: min=%lu max=%lu avg=%lu.%lu (n=%lu, target=10)\r\n",
			(unsigned long) dtMin, (unsigned long) dtMax,
			(unsigned long) (dtAvgX10 / 10u),
			(unsigned long) (dtAvgX10 % 10u),
			(unsigned long) dtN);
		Debug_Print(msg);
#endif

		/* Watchdog refresh ALWAYS runs — letting the IWDG time out (~8 s) and reset the MCU. */
		if (allAlive)
		{
			IWDG_Refresh();
		}
	}
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

	UartTx_Init();

	printf("Serial driver started\r\n");

#if DIAG_TEST
	/* On-device CRC16-CCITT self-test against the standard "123456789" vector. */
	{
		static const uint8_t crcVec[9] = {'1','2','3','4','5','6','7','8','9'};
		uint16_t crcChk = Telemetry_Crc16Ccitt(crcVec, 9u);
		printf("[CRC16] \"123456789\" -> 0x%04X expected 0x29B1 -> %s\r\n",
		       crcChk, (crcChk == 0x29B1u) ? "PASS" : "FAIL");
	}
#endif

	xSensorQueue = xQueueCreateStatic(SENSOR_QUEUE_LENGTH, sizeof(ts_Bmi160_Data), ucSensorQueueStorage, &xSensorQueueCB);

	xDetectionQueue = xQueueCreateStatic(DETECTION_QUEUE_LENGTH, sizeof(ts_Processed_Data), ucDetectionQueueStorage, &xDetectionQueueCB);

	xBmi160DataReadySem = xSemaphoreCreateBinaryStatic(&xBmi160DataReadySemBuffer);

	if (xSensorQueue == NULL || xDetectionQueue == NULL || xBmi160DataReadySem == NULL)
	{
		printf("FreeRTOS object creation failed!\r\n");
		Error_Handler();
	}

	xSensorTaskHandle = xTaskCreateStatic(vSensorReadTask, "SensRead", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, xSensorTaskStack, &xSensorTaskTCB);

	xProcessingTaskHandle = xTaskCreateStatic(vDataProcessingTask, "DataProc", PROCESSING_TASK_STACK_SIZE, NULL, PROCESSING_TASK_PRIORITY, xProcessingTaskStack, &xProcessingTaskTCB);

	xDetectionTaskHandle = xTaskCreateStatic(vCircleDetectionTask, "CircDet", DETECTION_TASK_STACK_SIZE, NULL, DETECTION_TASK_PRIORITY, xDetectionTaskStack, &xDetectionTaskTCB);

	xDiagTaskHandle = xTaskCreateStatic(vDiagTask, "Diag", DIAG_TASK_STACK_SIZE, NULL, DIAG_TASK_PRIORITY, xDiagTaskStack, &xDiagTaskTCB);

	if (xSensorTaskHandle == NULL || xProcessingTaskHandle == NULL ||
	    xDetectionTaskHandle == NULL || xDiagTaskHandle == NULL)
	{
		printf("Task creation failed!\r\n");
		Error_Handler();
	}

	IWDG_Init();

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
	 if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	    {
		UartTx_SendBytes((const uint8_t*) ptr, (uint16_t) len);
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
