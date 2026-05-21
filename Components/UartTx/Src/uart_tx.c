#include "uart_tx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "serial_driver.h"
#include <board.h>
#include <string.h>


#define UART_TX_QUEUE_LENGTH   32u
#define UART_TX_TASK_STACK     512u
#define UART_TX_TASK_PRIORITY  1u

typedef struct
{
	uint8_t  msg[UART_TX_MSG_MAX];
	uint16_t len;
} UartTx_Msg_t;


static QueueHandle_t s_queue       = NULL;
static StaticQueue_t s_queueCB;
static uint8_t       s_queueStorage[UART_TX_QUEUE_LENGTH * sizeof(UartTx_Msg_t)];

static TaskHandle_t  s_taskHandle  = NULL;
static StaticTask_t  s_taskCB;
static StackType_t   s_taskStack[UART_TX_TASK_STACK];

static void vUartTxWorker(void *argument);


void UartTx_Init(void)
{
	if (s_queue != NULL)
	{
		return;  
	}

	s_queue = xQueueCreateStatic(UART_TX_QUEUE_LENGTH, sizeof(UartTx_Msg_t),
	                             s_queueStorage, &s_queueCB);

	s_taskHandle = xTaskCreateStatic(vUartTxWorker, "UartTx",
	                                 UART_TX_TASK_STACK, NULL,
	                                 UART_TX_TASK_PRIORITY,
	                                 s_taskStack, &s_taskCB);
}

void UartTx_SendBytes(const uint8_t *buf, uint16_t len)
{
	if (buf == NULL || len == 0u || s_queue == NULL)
	{
		return;
	}

	UartTx_Msg_t pkt;
	if (len > UART_TX_MSG_MAX)
	{
		len = UART_TX_MSG_MAX;
	}
	memcpy(pkt.msg, buf, len);
	pkt.len = len;

	/* Non-blocking — caller never waits on UART. */
	(void) xQueueSend(s_queue, &pkt, 0);
}

static void vUartTxWorker(void *argument)
{
	(void) argument;

	UartTx_Msg_t pkt;
	for (;;)
	{
		if (xQueueReceive(s_queue, &pkt, portMAX_DELAY) == pdTRUE)
		{
			SerialReadWrite_t txCtx;
			txCtx.index  = USER_SERIAL_INDEX;
			txCtx.ptr    = pkt.msg;
			txCtx.length = pkt.len;

			if (Serial_Write_DMA(&txCtx, pkt.len) == E_SERIAL_ERR_NONE)
			{
				ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			}
		}
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		if (s_taskHandle != NULL)
		{
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			vTaskNotifyGiveFromISR(s_taskHandle, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}
