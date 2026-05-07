#ifndef _VIMU_RING_BUFFER_H_
#define _VIMU_RING_BUFFER_H_

#include <stdint.h>
#include <stdio.h>

typedef enum
{
    VIMU_RING_BUFFER_EMTY = 0,
    VIMU_RING_BUFFER_1S  = 1,
    VIMU_RING_BUFFER_2S  = 2,
    VIMU_RING_BUFFER_3S  = 3,
    VIMU_RING_BUFFER_4S  = 4,
    VIMU_RING_BUFFER_FULL
} vimu_ring_buffer_type_t;

typedef struct
{
	volatile uint16_t head;
	volatile uint16_t tail;
	uint16_t size;
	uint8_t *buffer;
} vimu_ring_buffer_t;

void vimu_ring_buffer_init(vimu_ring_buffer_t *ringBuffer, uint8_t *storage, uint16_t size);
void vimu_ring_buffer_clear(vimu_ring_buffer_t *ringBuffer);
uint8_t vimu_ring_buffer_put(vimu_ring_buffer_t *ringBuffer, uint8_t data);
uint8_t vimu_ring_buffer_get(vimu_ring_buffer_t *ringBuffer, uint8_t *data);
uint16_t vimu_ring_buffer_available(const vimu_ring_buffer_t *ringBuffer);
uint8_t vimu_ring_buffer_is_empty(const vimu_ring_buffer_t *ringBuffer);
uint8_t vimu_ring_buffer_is_full(const vimu_ring_buffer_t *ringBuffer);

#endif /* _VIMU_RING_BUFFER_H_ */
