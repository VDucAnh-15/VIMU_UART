#ifndef VIMU_FIFO_H
#define VIMU_FIFO_H

#include <stdint.h>

/* One sample = X/Y/Z, each axis stored on 16 bits. */
#define VIMU_UART_SAMPLE_SIZE      6U
#define VIMU_FIFO_SAMPLE_CAPACITY  32U

typedef struct
{
    uint8_t bytes[VIMU_UART_SAMPLE_SIZE];
} vimu_sample_t;

typedef struct
{
    vimu_sample_t buffer[VIMU_FIFO_SAMPLE_CAPACITY];
    vimu_sample_t currentSample;
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} vimu_fifo_t;

void vimu_sample_clear(vimu_sample_t *sample);
void vimu_fifo_clear(vimu_fifo_t *fifo);
uint8_t vimu_fifo_is_empty(const vimu_fifo_t *fifo);
uint8_t vimu_fifo_is_full(const vimu_fifo_t *fifo);
uint8_t vimu_fifo_push(vimu_fifo_t *fifo, const uint8_t *sampleBytes);
uint8_t vimu_fifo_pop(vimu_fifo_t *fifo, vimu_sample_t *sampleOut);
uint8_t vimu_fifo_sample_count(const vimu_fifo_t *fifo);

#endif /* VIMU_FIFO_H */
