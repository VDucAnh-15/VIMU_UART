#include "vimu_ring_buffer.h"

static uint16_t vimu_ring_buffer_next_index(const vimu_ring_buffer_t *ringBuffer, uint16_t index)
{
	index++;

	if (index >= ringBuffer->size)
	{
		index = 0U;
	}

	return index;
}

void vimu_ring_buffer_init(vimu_ring_buffer_t *ringBuffer, uint8_t *storage, uint16_t size)
{
	if ((ringBuffer == 0) || (storage == 0) || (size < 2U))
	{
		return;
	}

	ringBuffer->head = 0U;
	ringBuffer->tail = 0U;
	ringBuffer->size = size;
	ringBuffer->buffer = storage;
}

void vimu_ring_buffer_clear(vimu_ring_buffer_t *ringBuffer)
{
	if (ringBuffer == 0)
	{
		return;
	}

	ringBuffer->head = 0U;
	ringBuffer->tail = 0U;
}

uint8_t vimu_ring_buffer_put(vimu_ring_buffer_t *ringBuffer, uint8_t data)
{
	uint16_t nextHead;

	if ((ringBuffer == 0) || (ringBuffer->buffer == 0))
	{
		return 0U;
	}

	nextHead = vimu_ring_buffer_next_index(ringBuffer, ringBuffer->head);

	if (nextHead == ringBuffer->tail)
	{
		return 0U;
	}

	ringBuffer->buffer[ringBuffer->head] = data;
	ringBuffer->head = nextHead;

	return 1U;
}

uint8_t vimu_ring_buffer_get(vimu_ring_buffer_t *ringBuffer, uint8_t *data)
{
	if ((ringBuffer == 0) || (ringBuffer->buffer == 0) || (data == 0))
	{
		return 0U;
	}

	if (ringBuffer->head == ringBuffer->tail)
	{
		return 0U;
	}

	*data = ringBuffer->buffer[ringBuffer->tail];
	ringBuffer->tail = vimu_ring_buffer_next_index(ringBuffer, ringBuffer->tail);

	return 1U;
}

uint16_t vimu_ring_buffer_available(const vimu_ring_buffer_t *ringBuffer)
{
	if ((ringBuffer == 0) || (ringBuffer->buffer == 0))
	{
		return 0U;
	}

	if (ringBuffer->head >= ringBuffer->tail)
	{
		return (uint16_t)(ringBuffer->head - ringBuffer->tail);
	}

	return (uint16_t)(ringBuffer->size - (ringBuffer->tail - ringBuffer->head));
}

uint8_t vimu_ring_buffer_is_empty(const vimu_ring_buffer_t *ringBuffer)
{
	if (ringBuffer == 0)
	{
		return 1U;
	}

	return (uint8_t)(ringBuffer->head == ringBuffer->tail);
}

uint8_t vimu_ring_buffer_is_full(const vimu_ring_buffer_t *ringBuffer)
{
	uint16_t nextHead;

	if ((ringBuffer == 0) || (ringBuffer->buffer == 0))
	{
		return 0U;
	}

	nextHead = vimu_ring_buffer_next_index(ringBuffer, ringBuffer->head);

	return (uint8_t)(nextHead == ringBuffer->tail);
}

