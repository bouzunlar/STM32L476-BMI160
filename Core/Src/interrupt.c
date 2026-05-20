#include "interrupt.h"
#include <stddef.h>

typedef struct
{
	interrupt_handler_t handler;
	void               *context;
	uint32_t            count;  
} interrupt_slot_t;

static interrupt_slot_t interrupt_table[INTERRUPT_ID_MAX];

void Interrupt_Init(void)
{
	for (uint32_t i = 0u; i < (uint32_t) INTERRUPT_ID_MAX; i++)
	{
		interrupt_table[i].handler = NULL;
		interrupt_table[i].context = NULL;
		interrupt_table[i].count   = 0u;
	}
}

interrupt_err_t Interrupt_Register(interrupt_id_t id, interrupt_handler_t handler, void *context)
{
	if (id >= INTERRUPT_ID_MAX)
	{
		return INTERRUPT_ERR_INVALID_ID;
	}
	if (handler == NULL)
	{
		return INTERRUPT_ERR_NULL_HANDLER;
	}

	interrupt_table[id].handler = handler;
	interrupt_table[id].context = context;
	return INTERRUPT_ERR_NONE;
}

interrupt_err_t Interrupt_Unregister(interrupt_id_t id)
{
	if (id >= INTERRUPT_ID_MAX)
	{
		return INTERRUPT_ERR_INVALID_ID;
	}

	interrupt_table[id].handler = NULL;
	interrupt_table[id].context = NULL;
	return INTERRUPT_ERR_NONE;
}

void Interrupt_Dispatch(interrupt_id_t id)
{
	if (id >= INTERRUPT_ID_MAX)
	{
		return;
	}

	interrupt_table[id].count++;

	if (interrupt_table[id].handler != NULL)
	{
		interrupt_table[id].handler(interrupt_table[id].context);
	}
}

uint32_t Interrupt_GetCount(interrupt_id_t id)
{
	if (id >= INTERRUPT_ID_MAX)
	{
		return 0u;
	}
	return interrupt_table[id].count;
}
