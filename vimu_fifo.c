#include "vimu_fifo.h"

void vimu_sample_clear(vimu_sample_t *sample)
{
    uint8_t idx;

    if (sample == 0)
    {
        return;
    }

    for (idx = 0U; idx < VIMU_UART_SAMPLE_SIZE; idx++)
    {
        sample->bytes[idx] = 0U;
    }
}

void vimu_fifo_clear(vimu_fifo_t *fifo)
{
    if (fifo == 0)
    {
        return;
    }

    fifo->head = 0U;
    fifo->tail = 0U;
    fifo->count = 0U;
    vimu_sample_clear(&fifo->currentSample);
}

uint8_t vimu_fifo_is_empty(const vimu_fifo_t *fifo)
{
    if (fifo == 0)
    {
        return 1U;
    }

    return (uint8_t)(fifo->count == 0U);
}

uint8_t vimu_fifo_is_full(const vimu_fifo_t *fifo)
{
    if (fifo == 0)
    {
        return 0U;
    }

    return (uint8_t)(fifo->count >= VIMU_FIFO_SAMPLE_CAPACITY);
}

uint8_t vimu_fifo_push(vimu_fifo_t *fifo, const uint8_t *sampleBytes)
{
    uint8_t idx;
    uint8_t insertIndex;

    if ((fifo == 0) || (sampleBytes == 0) || (vimu_fifo_is_full(fifo) != 0U))
    {
        return 0U;
    }

    insertIndex = fifo->head;

    for (idx = 0U; idx < VIMU_UART_SAMPLE_SIZE; idx++)
    {
        fifo->buffer[insertIndex].bytes[idx] = sampleBytes[idx];
    }

    insertIndex++;
    if (insertIndex >= VIMU_FIFO_SAMPLE_CAPACITY)
    {
        insertIndex = 0U;
    }

    fifo->head = insertIndex;
    fifo->count++;

    return 1U;
}

uint8_t vimu_fifo_pop(vimu_fifo_t *fifo, vimu_sample_t *sampleOut)
{
    uint8_t idx;
    uint8_t removeIndex;

    if ((fifo == 0) || (sampleOut == 0) || (vimu_fifo_is_empty(fifo) != 0U))
    {
        return 0U;
    }

    removeIndex = fifo->tail;

    for (idx = 0U; idx < VIMU_UART_SAMPLE_SIZE; idx++)
    {
        sampleOut->bytes[idx] = fifo->buffer[removeIndex].bytes[idx];
        fifo->currentSample.bytes[idx] = sampleOut->bytes[idx];
    }

    removeIndex++;
    if (removeIndex >= VIMU_FIFO_SAMPLE_CAPACITY)
    {
        removeIndex = 0U;
    }

    fifo->tail = removeIndex;
    fifo->count--;

    return 1U;
}

uint8_t vimu_fifo_peek(const vimu_fifo_t *fifo, vimu_sample_t *sampleOut)
{
    uint8_t idx;
    uint8_t sampleIndex;

    if ((fifo == 0) || (sampleOut == 0) || (vimu_fifo_is_empty(fifo) != 0U))
    {
        return 0U;
    }

    sampleIndex = fifo->tail;

    for (idx = 0U; idx < VIMU_UART_SAMPLE_SIZE; idx++)
    {
        sampleOut->bytes[idx] = fifo->buffer[sampleIndex].bytes[idx];
    }

    return 1U;
}

uint8_t vimu_fifo_sample_count(const vimu_fifo_t *fifo)
{
    if (fifo == 0)
    {
        return 0U;
    }

    return fifo->count;
}
