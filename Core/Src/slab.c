#include "slab.h"

typedef struct
{
	uint8_t slab[SLAB_SMALL_SIZE];
	bool inUse;
} SmallSlab_t;

typedef struct
{
	uint8_t slab[SLAB_LARGE_SIZE];
	bool inUse;
} LargeSlab_t;

static SmallSlab_t smallSlabs[SLAB_SMALL_COUNT];
static LargeSlab_t largeSlabs[SLAB_LARGE_COUNT];

void Slab_Init()
{
	for (int i = 0; i < SLAB_SMALL_COUNT; i++)
	{
		smallSlabs[i].inUse = false;
	}
	for (int i = 0; i < SLAB_LARGE_COUNT; i++)
	{
		largeSlabs[i].inUse = false;
	}
}

void* Slab_Alloc(size_t size)
{
	if (size <= SLAB_SMALL_SIZE)
	{
		for (int i = 0; i < SLAB_SMALL_COUNT; i++)
		{
			if (!smallSlabs[i].inUse)
			{
				smallSlabs[i].inUse = true;
				return smallSlabs[i].slab;
			}
		}
	}
	else if (size <= SLAB_LARGE_SIZE)
	{
		for (int i = 0; i < SLAB_LARGE_COUNT; i++)
		{
			if (!largeSlabs[i].inUse)
			{
				largeSlabs[i].inUse = true;
				return largeSlabs[i].slab;
			}
		}
	}
	return NULL; // No slab available
}

void Slab_Free(void *ptr)
{
// Check if pointer belongs to small slabs
	for (int i = 0; i < SLAB_SMALL_COUNT; i++)
	{
		if (ptr == smallSlabs[i].slab)
		{
			smallSlabs[i].inUse = false;
			return;
		}
	}
// Check if pointer belongs to large slabs
	for (int i = 0; i < SLAB_LARGE_COUNT; i++)
	{
		if (ptr == largeSlabs[i].slab)
		{
			largeSlabs[i].inUse = false;
			return;
		}
	}
// Pointer not found (invalid free)
}

