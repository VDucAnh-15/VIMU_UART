#include "stm32f10x.h"
#include "vimu_app.h"
#include "vimu_ring_buffer.h"
#include "vimu_timer.h"
#include "vimu_uart.h"

typedef struct
{
    vimu_ring_buffer_t rxDataRing;
    uint8_t ringStorage[VIMU_RING_STORAGE_SIZE];
    vimu_fifo_t fifo;
    volatile vimu_state_t state;
    volatile uint8_t linkReady;
    volatile uint8_t requestMoreDataPending;
    volatile uint8_t requestMoreDataLatched;
    volatile uint16_t pendingDrainTicks;
} vimu_uart_app_t;

static vimu_uart_app_t g_vimuApp;

static uint8_t vimu_fifo_transfer_one_from_ring(void);
static uint16_t vimu_app_ring_sample_count(void);
static uint16_t vimu_app_total_sample_count(void);
static uint16_t vimu_app_free_sample_count(void);
static vimu_buf_level_t vimu_app_get_buffer_level(uint16_t sampleCount);
static uint8_t vimu_app_get_status_flags(void);
static void vimu_app_build_status_payload(uint8_t *payload);
static void vimu_app_update_low_watermark(void);
static void vimu_uart_send_frame(uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t payloadLength);
static void vimu_uart_send_ack(uint8_t cmd);
static void vimu_uart_send_nack(uint8_t cmd, uart_error_t errorCode);
static void vimu_uart_send_status_response(uint8_t cmd);
static void vimu_uart_send_request_more_data(void);
static void vimu_handle_frame(const frame_Message_t *rxFrame);
static void vimu_drain_one_sample(void);

static uint8_t vimu_fifo_transfer_one_from_ring(void)
{
    uint8_t sampleBytes[VIMU_UART_SAMPLE_SIZE];
    uint16_t readCount;

    if ((vimu_fifo_is_full(&g_vimuApp.fifo) != 0U) || (vimu_app_ring_sample_count() == 0U))
    {
        return 0U;
    }

    readCount = vimu_ring_buffer_read(&g_vimuApp.rxDataRing, sampleBytes, VIMU_UART_SAMPLE_SIZE);
    if (readCount != VIMU_UART_SAMPLE_SIZE)
    {
        return 0U;
    }

    return vimu_fifo_push(&g_vimuApp.fifo, sampleBytes);
}

void vimu_app_init(void)
{
    vimu_ring_buffer_init(&g_vimuApp.rxDataRing, g_vimuApp.ringStorage, VIMU_RING_STORAGE_SIZE);
    vimu_fifo_clear(&g_vimuApp.fifo);
    g_vimuApp.state = VIMU_STATE_STOPPED;
    g_vimuApp.linkReady = 0U;
    g_vimuApp.requestMoreDataPending = 0U;
    g_vimuApp.requestMoreDataLatched = 0U;
    g_vimuApp.pendingDrainTicks = 0U;
}

static uint16_t vimu_app_ring_sample_count(void)
{
    return (uint16_t)(vimu_ring_buffer_available(&g_vimuApp.rxDataRing) / VIMU_UART_SAMPLE_SIZE);
}

static uint16_t vimu_app_total_sample_count(void)
{
    return (uint16_t)(vimu_app_ring_sample_count() + vimu_fifo_sample_count(&g_vimuApp.fifo));
}

static uint16_t vimu_app_free_sample_count(void)
{
    uint16_t totalSamples;

    totalSamples = vimu_app_total_sample_count();
    if (totalSamples >= VIMU_TOTAL_SAMPLE_CAPACITY)
    {
        return 0U;
    }

    return (uint16_t)(VIMU_TOTAL_SAMPLE_CAPACITY - totalSamples);
}

static vimu_buf_level_t vimu_app_get_buffer_level(uint16_t sampleCount)
{
    if (sampleCount == 0U)
    {
        return VIMU_BUF_EMPTY;
    }

    if (sampleCount <= 50U)
    {
        return VIMU_BUF_1S;
    }

    if (sampleCount <= 100U)
    {
        return VIMU_BUF_2S;
    }

    if (sampleCount <= 150U)
    {
        return VIMU_BUF_3S;
    }

    if (sampleCount <= 200U)
    {
        return VIMU_BUF_4S;
    }

    return VIMU_BUF_FULL;
}

static uint8_t vimu_app_get_status_flags(void)
{
    uint8_t flags;
    uint16_t ringSamples;

    flags = 0U;
    ringSamples = vimu_app_ring_sample_count();

    if (ringSamples < VIMU_LOW_WATERMARK_SAMPLES)
    {
        flags |= VIMU_FLAG_BUFFER_LOW;
    }

    if (vimu_fifo_is_full(&g_vimuApp.fifo) != 0U)
    {
        flags |= VIMU_FLAG_FIFO_FULL;
        flags |= VIMU_FLAG_IRQ_ACTIVE;
    }

    if (g_vimuApp.state == VIMU_STATE_RUNNING)
    {
        flags |= VIMU_FLAG_I2C_ENABLE;
    }

    if (ringSamples == 0U)
    {
        flags |= VIMU_FLAG_RING_EMPTY;
    }

    return flags;
}

static void vimu_app_build_status_payload(uint8_t *payload)
{
    uint16_t ringSamples;

    if (payload == 0)
    {
        return;
    }

    ringSamples = vimu_app_ring_sample_count();

    payload[0] = (uint8_t)g_vimuApp.state;
    payload[1] = (uint8_t)vimu_app_get_buffer_level(ringSamples);
    payload[2] = vimu_app_get_status_flags();
}

static void vimu_app_update_low_watermark(void)
{
    uint16_t ringSamples;

    ringSamples = vimu_app_ring_sample_count();

    if ((g_vimuApp.linkReady != 0U) && (g_vimuApp.state == VIMU_STATE_RUNNING))
    {
        if ((ringSamples < VIMU_LOW_WATERMARK_SAMPLES) && (g_vimuApp.requestMoreDataLatched == 0U))
        {
            g_vimuApp.requestMoreDataPending = 1U;
            g_vimuApp.requestMoreDataLatched = 1U;
        }
        else if (ringSamples >= VIMU_LOW_WATERMARK_SAMPLES)
        {
            g_vimuApp.requestMoreDataLatched = 0U;
        }
    }
    else
    {
        g_vimuApp.requestMoreDataPending = 0U;
        g_vimuApp.requestMoreDataLatched = 0U;
    }
}

static void vimu_uart_send_frame(uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t payloadLength)
{
    static frame_Message_t txFrame;
    static uint8_t txBytes[UART_PROTO_MAX_FRAME_LEN];
    uint16_t idx;
    uint16_t txLength;

    txFrame.startFrame = 0U;
    txFrame.typeMessage = type;
    txFrame.cmdCode = cmd;
    txFrame.lengthData = payloadLength;
    txFrame.crc = 0U;

    for (idx = 0U; idx < payloadLength; idx++)
    {
        txFrame.data[idx] = payload[idx];
    }

    txLength = Message_Create_Frame(&txFrame, txBytes);
    if (txLength != 0U)
    {
        vimu_uart_send_bytes(USART1, txBytes, txLength);
    }
}

static void vimu_uart_send_ack(uint8_t cmd)
{
    vimu_uart_send_frame(VIMU_UART_PKT_TYPE_ACK, cmd, 0, 0U);
}

static void vimu_uart_send_nack(uint8_t cmd, uart_error_t errorCode)
{
    uint8_t payload[1];

    payload[0] = (uint8_t)errorCode;
    vimu_uart_send_frame(VIMU_UART_PKT_TYPE_NACK, cmd, payload, 1U);
}

static void vimu_uart_send_status_response(uint8_t cmd)
{
    uint8_t payload[VIMU_STATUS_PAYLOAD_SIZE];

    vimu_app_build_status_payload(payload);
    vimu_uart_send_frame(VIMU_UART_PKT_TYPE_RSP, cmd, payload, VIMU_STATUS_PAYLOAD_SIZE);
}

static void vimu_uart_send_request_more_data(void)
{
    vimu_uart_send_frame(VIMU_UART_PKT_TYPE_EVT, VIMU_UART_CMD_REQ_MORE_DATA, 0, 0U);
}

static void vimu_handle_frame(const frame_Message_t *rxFrame)
{
    uint16_t sampleCount;
    uint16_t writtenCount;

    if (rxFrame == 0)
    {
        return;
    }

    if (rxFrame->typeMessage != VIMU_UART_PKT_TYPE_CMD)
    {
        vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_FORMAT);
        return;
    }

    if ((rxFrame->cmdCode != VIMU_UART_CMD_HANDSHAKE) && (g_vimuApp.linkReady == 0U))
    {
        vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_INVALID_STATE);
        return;
    }

    switch (rxFrame->cmdCode)
    {
        case VIMU_UART_CMD_HANDSHAKE:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            g_vimuApp.linkReady = 1U;
            vimu_uart_send_status_response(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_NEW_DATA:
            if ((rxFrame->lengthData == 0U) ||
                ((rxFrame->lengthData % VIMU_UART_SAMPLE_SIZE) != 0U))
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            sampleCount = (uint16_t)(rxFrame->lengthData / VIMU_UART_SAMPLE_SIZE);
            if (sampleCount > vimu_app_free_sample_count())
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BUFFER_FULL);
                break;
            }

            writtenCount = vimu_ring_buffer_write(&g_vimuApp.rxDataRing,
                                                  rxFrame->data,
                                                  rxFrame->lengthData);
            if (writtenCount != rxFrame->lengthData)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BUFFER_FULL);
                break;
            }

            if ((g_vimuApp.state == VIMU_STATE_RUNNING) && (vimu_fifo_is_empty(&g_vimuApp.fifo) != 0U))
            {
                (void)vimu_fifo_transfer_one_from_ring();
            }
            vimu_app_update_low_watermark();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_CLEAR:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            vimu_ring_buffer_clear(&g_vimuApp.rxDataRing);
            vimu_fifo_clear(&g_vimuApp.fifo);
            g_vimuApp.pendingDrainTicks = 0U;
            vimu_app_update_low_watermark();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_STOP:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            g_vimuApp.state = VIMU_STATE_STOPPED;
            g_vimuApp.pendingDrainTicks = 0U;
            vimu_app_update_low_watermark();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_START:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            g_vimuApp.state = VIMU_STATE_RUNNING;
            g_vimuApp.pendingDrainTicks = 0U;
            if (vimu_fifo_is_empty(&g_vimuApp.fifo) != 0U)
            {
                (void)vimu_fifo_transfer_one_from_ring();
            }
            vimu_app_update_low_watermark();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_UPDATE:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            vimu_uart_send_status_response(rxFrame->cmdCode);
            break;

        default:
            vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_UNKNOWN_CMD);
            break;
    }
}

void vimu_app_process_uart_frame(void)
{
    static uint8_t rxBytes[UART_PROTO_MAX_FRAME_LEN];
    static frame_Message_t rxFrame;
    uint16_t rxLength;

    rxLength = 0U;

    if (vimu_uart_receive_frame_bytes(USART1, rxBytes, &rxLength, UART_PROTO_MAX_FRAME_LEN) == 0U)
    {
        return;
    }

    if (Message_Detect_Frame(rxBytes, &rxFrame) == 0U)
    {
        if (rxLength >= UART_PROTO_FIXED_SIZE)
        {
            vimu_uart_send_nack(rxBytes[3], VIMU_UART_ERR_BAD_CRC);
        }
        return;
    }

    vimu_handle_frame(&rxFrame);
}

static void vimu_drain_one_sample(void)
{
    vimu_sample_t activeSample;

    if (g_vimuApp.state != VIMU_STATE_RUNNING)
    {
        return;
    }

    if (vimu_fifo_pop(&g_vimuApp.fifo, &activeSample) == 0U)
    {
        vimu_app_update_low_watermark();
        return;
    }

    (void)vimu_fifo_transfer_one_from_ring();
    vimu_app_update_low_watermark();
}

void vimu_app_service_runtime(void)
{
    uint16_t pendingTicks;

    __disable_irq();
    pendingTicks = g_vimuApp.pendingDrainTicks;
    g_vimuApp.pendingDrainTicks = 0U;
    __enable_irq();

    while (pendingTicks != 0U)
    {
        vimu_drain_one_sample();
        pendingTicks--;
    }
}

void vimu_app_service_events(void)
{
    if (g_vimuApp.requestMoreDataPending != 0U)
    {
        g_vimuApp.requestMoreDataPending = 0U;
        vimu_uart_send_request_more_data();
    }
}

void vimu_timer_tick_callback(void)
{
    if (g_vimuApp.state != VIMU_STATE_RUNNING)
    {
        return;
    }

    if (g_vimuApp.pendingDrainTicks < 0xFFFFU)
    {
        g_vimuApp.pendingDrainTicks++;
    }
}
