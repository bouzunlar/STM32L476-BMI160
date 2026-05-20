#include "serial_driver.h"
#include "serial_hal.h"
#include <stdint.h>

static void Serial_vInit(void);
static void Serial_vDisable(uint8_t index);
static void Serial_vIndividualInit(uint8_t cnt);
static void Serial_vIndividualInitDMA(uint8_t cnt);

UART_HandleTypeDef huart[SERIAL_TOTAL_NUM];
DMA_HandleTypeDef hdma_usart2_tx;

static uint8_t serialDmaEnabled[SERIAL_TOTAL_NUM];

serialInterrupt_t serial_irq_context[SERIAL_MAX_INTERRUPT_HANDLERS];

serialErrorCodes_t Serial_Open(void *vpParam)
{
	uint8_t index = *(uint8_t*) vpParam;
	if (SERIAL_TOTAL_NUM < index)
	{
		return E_SERIAL_ERR_WRONG_INDEX;
	}
	else if (SERIAL_TOTAL_NUM > index)
	{
		Serial_vIndividualInit(index);
	}
	else if (SERIAL_TOTAL_NUM == index)
	{
		Serial_vInit();
	}
	return E_SERIAL_ERR_NONE;
}

serialErrorCodes_t Serial_Ioctl(SERIAL_IOCTL_COMMANDS_T eCommand, void *vpParam)
{
	switch (eCommand)
	{
	case E_SERIAL_IOCTL_GET_VERSION:
		*(float*) vpParam = SERIAL_MODULE_SW_VERSION;
		break;
	case E_SERIAL_IOCTL_ENABLE_DMA_TX:
	{
		uint8_t index = *(uint8_t*) vpParam;
		if (index >= SERIAL_TOTAL_NUM)
		{
			return E_SERIAL_ERR_WRONG_INDEX;
		}
		if (serialDmaEnabled[index] == 0u)
		{
			Serial_vIndividualInitDMA(index);
		}
		break;
	}
	case E_SERIAL_IOCTL_ENABLE_INTERRUPT:
	{
		serialInterrupt_t sSerialInterrupt = *(serialInterrupt_t*) vpParam;
		if (sSerialInterrupt.index >= SERIAL_TOTAL_NUM)
		{
			return E_SERIAL_ERR_WRONG_INDEX;
		}
		__HAL_UART_ENABLE_IT(&huart[sSerialInterrupt.index], UART_IT_RXNE);
		printf("[SERIAL] RX interrupt activated (index=%d)\r\n", sSerialInterrupt.index);
		break;
	}
	case E_SERIAL_IOCTL_DISABLE_INTERRUPT:
	{
		serialInterrupt_t sSerialInterrupt = *(serialInterrupt_t*) vpParam;
		if (sSerialInterrupt.index >= SERIAL_TOTAL_NUM)
		{
			return E_SERIAL_ERR_WRONG_INDEX;
		}
		__HAL_UART_DISABLE_IT(&huart[sSerialInterrupt.index], UART_IT_RXNE);
		printf("RX interrupt disabled (index=%d)\r\n", sSerialInterrupt.index);
		break;
	}
	case E_SERIAL_IOCTL_REGISTER_IRQ_HANDLER:
	{
		serialInterrupt_t sSerialInterrupt = *(serialInterrupt_t*) vpParam;
		if (sSerialInterrupt.index >= SERIAL_TOTAL_NUM)
		{
			return E_SERIAL_ERR_WRONG_INDEX;
		}
		serial_irq_context[sSerialInterrupt.index].index = sSerialInterrupt.index;
		serial_irq_context[sSerialInterrupt.index].interrupt_handler = sSerialInterrupt.interrupt_handler;
		serial_irq_context[sSerialInterrupt.index].interrupt_id = sSerialInterrupt.interrupt_id;
		serial_irq_context[sSerialInterrupt.index].isInterruptOccured = sSerialInterrupt.isInterruptOccured;
		serial_irq_context[sSerialInterrupt.index].type = sSerialInterrupt.type;
		break;
	}
	default:
		return E_SERIAL_ERR_WRONG_IOCTL_CMD;
		break;
	}
	return E_SERIAL_ERR_NONE;
}

serialErrorCodes_t Serial_Write(const void *pvBuffer, const uint32_t xBytes)
{
	SerialReadWrite_t *serialWrite = (SerialReadWrite_t*) pvBuffer;
	if (serialWrite->index >= SERIAL_TOTAL_NUM)
	{
		return E_SERIAL_ERR_WRONG_INDEX;
	}
	if (serialWrite->ptr == NULL || serialWrite->length == 0u)
	{
		return E_SERIAL_ERR_HW_ERROR;
	}

	if (HAL_UART_Transmit(&huart[serialWrite->index], (uint8_t*) serialWrite->ptr, serialWrite->length,
	HAL_MAX_DELAY) != HAL_OK)
	{
		printf("TX Error (index=%d)\r\n", serialWrite->index);
		return E_SERIAL_ERR_HW_ERROR;
	}

	return E_SERIAL_ERR_NONE;
}

serialErrorCodes_t Serial_Write_DMA(const void *pvBuffer, const uint32_t xBytes)
{
	SerialReadWrite_t *serialWrite = (SerialReadWrite_t*) pvBuffer;
	if (serialWrite->index >= SERIAL_TOTAL_NUM)
	{
		return E_SERIAL_ERR_WRONG_INDEX;
	}
	if (serialWrite->ptr == NULL || serialWrite->length == 0u)
	{
		return E_SERIAL_ERR_HW_ERROR;
	}
	if (serialDmaEnabled[serialWrite->index] == 0u)
	{
		return E_SERIAL_ERR_DMA_NOT_ENABLED;
	}
	if (huart[serialWrite->index].gState != HAL_UART_STATE_READY)
	{
		return E_SERIAL_ERR_HW_ERROR;
	}

	if (HAL_UART_Transmit_DMA(&huart[serialWrite->index], (uint8_t*) serialWrite->ptr, serialWrite->length) != HAL_OK)
	{
		return E_SERIAL_ERR_HW_ERROR;
	}

	return E_SERIAL_ERR_NONE;
}

serialErrorCodes_t Serial_Read(const void *pvBuffer, const uint32_t xBytes)
{
	SerialReadWrite_t *serialRead = (SerialReadWrite_t*) pvBuffer;

	if (serialRead->index >= SERIAL_TOTAL_NUM)
	{
		return E_SERIAL_ERR_WRONG_INDEX;
	}
	if (serialRead->ptr == NULL || serialRead->length == 0u)
	{
		return E_SERIAL_ERR_HW_ERROR;
	}

	if (HAL_UART_Receive(&huart[serialRead->index], (uint8_t*) serialRead->ptr, serialRead->length,
	HAL_MAX_DELAY) != HAL_OK)
	{
		printf(" RX Error (index=%d)\r\n", serialRead->index);
		return E_SERIAL_ERR_HW_ERROR;
	}

	return E_SERIAL_ERR_NONE;
}

serialErrorCodes_t Serial_Close(void *vpParam)
{
	uint8_t cnt;
	for (cnt = 0u; cnt < SERIAL_TOTAL_NUM; cnt++)
	{
		Serial_vDisable(cnt);
	}

	return E_SERIAL_ERR_NONE;
}

void Serial_IRQ_Dispatch(uint8_t index)
{
	if (index >= SERIAL_TOTAL_NUM)
	{
		return;
	}

	serial_irq_context[index].isInterruptOccured = 1u;

	if (serial_irq_context[index].interrupt_handler != NULL)
	{
		serial_irq_context[index].interrupt_handler((void*) (uintptr_t) serial_irq_context[index].interrupt_id);
	}
}

void Serial_DMA_TX_IRQHandler(uint8_t index)
{
	if (index < SERIAL_TOTAL_NUM)
	{
		HAL_DMA_IRQHandler(serialDmaHandleList[index]);
	}
}

static void Serial_vIndividualInit(uint8_t cnt)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	serialClockEnable();

	GPIO_InitStruct.Pin = serialPinList[cnt];
	GPIO_InitStruct.Mode = serialFunctionList[cnt];
	GPIO_InitStruct.Pull = serialPullList[cnt];
	GPIO_InitStruct.Speed = serialPinSpeedList[cnt];
	GPIO_InitStruct.Alternate = serialAfList[cnt];
	HAL_GPIO_Init((GPIO_TypeDef*) serialPortList[cnt], &GPIO_InitStruct);

	huart[cnt].Instance = (USART_TypeDef*) serialUsartList[cnt];
	huart[cnt].Init.BaudRate = serialBaudRateList[cnt];
	huart[cnt].Init.WordLength = UART_WORDLENGTH_8B;
	huart[cnt].Init.StopBits = UART_STOPBITS_1;
	huart[cnt].Init.Parity = UART_PARITY_NONE;
	huart[cnt].Init.Mode = UART_MODE_TX_RX;
	huart[cnt].Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart[cnt].Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart[cnt]) != HAL_OK)
	{
		printf("UART init error (index=%d)\r\n", cnt);
	}
	else
	{
		printf("UART init OK (index=%d, baud=%lu)\r\n", cnt, serialBaudRateList[cnt]);
	}
}

static void Serial_vIndividualInitDMA(uint8_t cnt)
{

	__HAL_RCC_DMA1_CLK_ENABLE();

	serialDmaHandleList[cnt]->Instance = (DMA_Channel_TypeDef*) serialDmaInstanceList[cnt];
	serialDmaHandleList[cnt]->Init.Request = serialDmaRequestList[cnt];
	serialDmaHandleList[cnt]->Init.Direction = DMA_MEMORY_TO_PERIPH;
	serialDmaHandleList[cnt]->Init.PeriphInc = DMA_PINC_DISABLE;
	serialDmaHandleList[cnt]->Init.MemInc = DMA_MINC_ENABLE;
	serialDmaHandleList[cnt]->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	serialDmaHandleList[cnt]->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	serialDmaHandleList[cnt]->Init.Mode = DMA_NORMAL;
	serialDmaHandleList[cnt]->Init.Priority = DMA_PRIORITY_LOW;

	if (HAL_DMA_Init(serialDmaHandleList[cnt]) != HAL_OK)
	{
		printf("DMA init error (index=%d)\r\n", cnt);
		return;
	}

	__HAL_LINKDMA(&huart[cnt], hdmatx, *serialDmaHandleList[cnt]);

	HAL_NVIC_SetPriority(serialDmaIrqList[cnt], SERIAL_DMA_IRQ_PRIORITY, 0u);
	HAL_NVIC_EnableIRQ(serialDmaIrqList[cnt]);

	HAL_NVIC_SetPriority(serialUartIrqList[cnt], SERIAL_UART_IRQ_PRIORITY, 0u);
	HAL_NVIC_EnableIRQ(serialUartIrqList[cnt]);

	serialDmaEnabled[cnt] = 1u;
	printf("DMA TX enabled (index=%d)\r\n", cnt);
}

static void Serial_vInit(void)
{
	uint8_t cnt;
	for (cnt = 0u; cnt < SERIAL_TOTAL_NUM; cnt++)
	{
		Serial_vIndividualInit(cnt);
	}
}

static void Serial_vDisable(uint8_t index)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	if (serialDmaEnabled[index] != 0u)
	{
		HAL_NVIC_DisableIRQ(serialUartIrqList[index]);
		HAL_NVIC_DisableIRQ(serialDmaIrqList[index]);
		HAL_DMA_DeInit(serialDmaHandleList[index]);
		serialDmaEnabled[index] = 0u;
	}

	HAL_UART_DeInit(&huart[index]);

	if (index == (SERIAL_TOTAL_NUM - 1u))
	{
		serialClockDisable();
	}

	GPIO_InitStruct.Pin = serialPinList[index];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = serialPullList[index];
	GPIO_InitStruct.Speed = serialPinSpeedList[index];
	HAL_GPIO_Init((GPIO_TypeDef*) serialPortList[index], &GPIO_InitStruct);

	printf("UART disabled (index=%d)\r\n", index);
}

int8_t SERIAL_TEST(uint32_t timeout)
{
	(void) timeout;

	uint8_t allIndex = SERIAL_TOTAL_NUM;
	Serial_Open(&allIndex);

	uint8_t testMsg[] = "SERIAL_TEST_OK\r\n";
	uint8_t failCount = 0u;

	for (uint8_t i = 0u; i < SERIAL_TOTAL_NUM; i++)
	{
		SerialReadWrite_t ctx;
		ctx.index = i;
		ctx.ptr = testMsg;
		ctx.length = (uint16_t) sizeof(testMsg) - 1u;

		if (Serial_Write(&ctx, ctx.length) != E_SERIAL_ERR_NONE)
		{
			printf("TEST FAILED (index=%d)\r\n", i);
			failCount++;
		}
		else
		{
			printf("TEST OK (index=%d)\r\n", i);
		}
		HAL_Delay(200u);
	}
	return (failCount == 0u) ? 0 : -1;
}
