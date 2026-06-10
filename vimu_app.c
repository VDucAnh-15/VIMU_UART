#include "stm32f10x.h"
#include "vimu_app.h"
#include "vimu_i2c_slave.h"
#include "vimu_ring_buffer.h"
#include "vimu_timer.h"
#include "vimu_uart.h"

typedef struct
{
    vimu_ring_buffer_t rxDataRing;
    uint8_t ringStorage[VIMU_RING_STORAGE_SIZE];
    vimu_fifo_t fifo;
    uint8_t fifoSnapshot[VIMU_I2C_FIFO_BLOCK_BYTES];
    volatile vimu_state_t state;
    volatile uint8_t linkReady;
    volatile uint8_t requestMoreDataPending;
    volatile uint8_t requestMoreDataLatched;
    volatile uint8_t fifoBlockReady;
    volatile uint8_t irqSignalActive;
    volatile uint8_t pendingI2cCommand;
    volatile uint8_t i2cCommandPending;
    volatile uint16_t pendingFillTicks;
} vimu_uart_app_t;

static vimu_uart_app_t g_vimuApp;
static volatile uint8_t g_vimuI2cRegisterPointer = VIMU_I2C_REG_STATE;
static volatile uint8_t g_vimuI2cCurrentRegister = VIMU_I2C_REG_STATE;
static volatile uint8_t g_vimuI2cRxCount = 0U;
static volatile uint8_t g_vimuI2cFifoReadActive = 0U;
static volatile uint8_t g_vimuI2cFifoReadComplete = 0U;
static volatile uint16_t g_vimuI2cFifoReadCount = 0U;

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
static void vimu_uart_send_status_push(void);
static void vimu_uart_send_request_more_data(void);
static void vimu_handle_frame(const frame_Message_t *rxFrame);
static void vimu_app_fill_fifo_one_sample(void);
static void vimu_app_prepare_fifo_snapshot(void);
static void vimu_app_raise_fifo_ready_irq(void);
static void vimu_app_release_fifo_block(void);
static void vimu_app_clear_buffers(void);
static void vimu_app_stop_streaming(void);
static void vimu_app_start_streaming(void);
static void vimu_app_queue_i2c_command(uint8_t command);
static void vimu_app_process_i2c_command(uint8_t command);
static void vimu_app_set_irq_signal(uint8_t active);

static uint8_t vimu_fifo_transfer_one_from_ring(void)
{
    uint8_t sampleBytes[VIMU_UART_SAMPLE_SIZE];
    uint16_t readCount;

    if ((vimu_fifo_is_full(&g_vimuApp.fifo) != 0U) ||
        (vimu_app_ring_sample_count() == 0U))
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
    g_vimuApp.fifoBlockReady = 0U;
    g_vimuApp.irqSignalActive = 0U;
    g_vimuApp.pendingI2cCommand = 0U;
    g_vimuApp.i2cCommandPending = 0U;
    g_vimuApp.pendingFillTicks = 0U;
    I2C_Slave_SetIrq(0U);
}

static void vimu_app_set_irq_signal(uint8_t active)
{
    g_vimuApp.irqSignalActive = (active != 0U) ? 1U : 0U;
    I2C_Slave_SetIrq(g_vimuApp.irqSignalActive);
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

    if ((g_vimuApp.fifoBlockReady != 0U) ||
        (vimu_fifo_is_full(&g_vimuApp.fifo) != 0U))
    {
        flags |= VIMU_FLAG_FIFO_FULL;
    }

    if (g_vimuApp.irqSignalActive != 0U)
    {
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

static void vimu_app_prepare_fifo_snapshot(void)
{
    uint8_t sampleIndex;
    uint16_t snapshotIndex;
    uint8_t sampleCount;
    uint8_t byteIndex;

    snapshotIndex = 0U;
    sampleIndex = g_vimuApp.fifo.tail;

    for (sampleCount = 0U; sampleCount < VIMU_FIFO_SAMPLE_CAPACITY; sampleCount++)
    {
        for (byteIndex = 0U; byteIndex < VIMU_UART_SAMPLE_SIZE; byteIndex++)
        {
            g_vimuApp.fifoSnapshot[snapshotIndex++] = g_vimuApp.fifo.buffer[sampleIndex].bytes[byteIndex];
        }

        sampleIndex++;
        if (sampleIndex >= VIMU_FIFO_SAMPLE_CAPACITY)
        {
            sampleIndex = 0U;
        }
    }
}

static void vimu_app_raise_fifo_ready_irq(void)
{
    if (vimu_fifo_is_full(&g_vimuApp.fifo) == 0U)
    {
        return;
    }

    vimu_app_prepare_fifo_snapshot();
    g_vimuApp.fifoBlockReady = 1U;
    vimu_fifo_clear(&g_vimuApp.fifo);
    vimu_app_set_irq_signal(1U);
    vimu_uart_send_status_push();
    vimu_app_set_irq_signal(0U);
    vimu_app_update_low_watermark();
}

static void vimu_app_release_fifo_block(void)
{
    if (g_vimuApp.fifoBlockReady == 0U)
    {
        return;
    }

    g_vimuApp.fifoBlockReady = 0U;
    vimu_app_set_irq_signal(0U);
    vimu_app_update_low_watermark();
    vimu_uart_send_status_push();
}

static void vimu_app_clear_buffers(void)
{
    vimu_ring_buffer_clear(&g_vimuApp.rxDataRing);
    vimu_fifo_clear(&g_vimuApp.fifo);
    g_vimuApp.fifoBlockReady = 0U;
    g_vimuApp.pendingFillTicks = 0U;
    vimu_app_set_irq_signal(0U);
    vimu_app_update_low_watermark();
    vimu_uart_send_status_push();
}

static void vimu_app_stop_streaming(void)
{
    g_vimuApp.state = VIMU_STATE_STOPPED;
    g_vimuApp.pendingFillTicks = 0U;
    vimu_app_set_irq_signal(0U);
    vimu_app_update_low_watermark();
}

static void vimu_app_start_streaming(void)
{
    g_vimuApp.state = VIMU_STATE_RUNNING;
    g_vimuApp.pendingFillTicks = 0U;
    vimu_app_raise_fifo_ready_irq();
    vimu_app_update_low_watermark();
}

static void vimu_app_queue_i2c_command(uint8_t command)
{
    g_vimuApp.pendingI2cCommand = command;
    g_vimuApp.i2cCommandPending = 1U;
}

static void vimu_app_process_i2c_command(uint8_t command)
{
    switch (command)
    {
        case VIMU_I2C_CMD_START:
            vimu_app_start_streaming();
            break;

        case VIMU_I2C_CMD_STOP:
            vimu_app_stop_streaming();
            break;

        case VIMU_I2C_CMD_CLEAR:
            vimu_app_clear_buffers();
            break;

        case VIMU_I2C_CMD_FIFO_RELEASE:
            vimu_app_release_fifo_block();
            break;

        default:
            break;
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

static void vimu_uart_send_status_push(void)
{
    uint8_t payload[VIMU_STATUS_PAYLOAD_SIZE];

    if (g_vimuApp.linkReady == 0U)
    {
        return;
    }

    vimu_app_build_status_payload(payload);
    vimu_uart_send_frame(VIMU_UART_PKT_TYPE_EVT,
                         VIMU_UART_CMD_STATUS_PUSH,
                         payload,
                         VIMU_STATUS_PAYLOAD_SIZE);
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

            vimu_app_raise_fifo_ready_irq();
            vimu_app_update_low_watermark();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_CLEAR:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            vimu_app_clear_buffers();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_STOP:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            vimu_app_stop_streaming();
            vimu_uart_send_ack(rxFrame->cmdCode);
            break;

        case VIMU_UART_CMD_START:
            if (rxFrame->lengthData != 0U)
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }

            vimu_app_start_streaming();
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

static void vimu_app_fill_fifo_one_sample(void)
{
    if (g_vimuApp.state != VIMU_STATE_RUNNING)
    {
        return;
    }

    if (vimu_fifo_transfer_one_from_ring() != 0U)
    {
        vimu_app_raise_fifo_ready_irq();
    }

    vimu_app_update_low_watermark();
}

void vimu_app_service_runtime(void)
{
    uint16_t pendingTicks;
    uint8_t i2cCommand;
    uint8_t hasI2cCommand;

    __disable_irq();
    pendingTicks = g_vimuApp.pendingFillTicks;
    g_vimuApp.pendingFillTicks = 0U;
    hasI2cCommand = g_vimuApp.i2cCommandPending;
    i2cCommand = g_vimuApp.pendingI2cCommand;
    g_vimuApp.i2cCommandPending = 0U;
    __enable_irq();

    if (hasI2cCommand != 0U)
    {
        vimu_app_process_i2c_command(i2cCommand);
    }

    while (pendingTicks != 0U)
    {
        vimu_app_fill_fifo_one_sample();
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

    if (g_vimuApp.pendingFillTicks < 0xFFFFU)
    {
        g_vimuApp.pendingFillTicks++;
    }
}

uint8_t vimu_app_i2c_read_register(uint8_t reg)
{
    uint16_t ringSamples;

    ringSamples = vimu_app_ring_sample_count();

    switch (reg)
    {
        case VIMU_I2C_REG_STATE:
            return (uint8_t)g_vimuApp.state;

        case VIMU_I2C_REG_BUFFER_LEVEL:
            return (uint8_t)vimu_app_get_buffer_level(ringSamples);

        case VIMU_I2C_REG_FLAGS:
            return vimu_app_get_status_flags();

        case VIMU_I2C_REG_FIFO_SAMPLE_COUNT:
            if (g_vimuApp.fifoBlockReady != 0U)
            {
                return VIMU_FIFO_SAMPLE_CAPACITY;
            }

            return vimu_fifo_sample_count(&g_vimuApp.fifo);

        case VIMU_I2C_REG_RING_SAMPLE_COUNT:
            return (uint8_t)ringSamples;

        case VIMU_I2C_REG_TOTAL_SAMPLE_COUNT:
            return (uint8_t)vimu_app_total_sample_count();

        case VIMU_I2C_REG_FIFO_BLOCK_READY:
            return g_vimuApp.fifoBlockReady;

        case VIMU_I2C_REG_FIFO_BLOCK_BYTES:
            return (uint8_t)VIMU_I2C_FIFO_BLOCK_BYTES;

        case VIMU_I2C_REG_PROTOCOL_VERSION:
            return VIMU_I2C_PROTOCOL_VERSION;

        default:
            break;
    }

    if ((reg >= VIMU_I2C_REG_FIFO_DATA_0) && (reg <= VIMU_I2C_REG_FIFO_DATA_LAST))
    {
        if (g_vimuApp.fifoBlockReady == 0U)
        {
            return 0U;
        }

        return g_vimuApp.fifoSnapshot[(uint16_t)(reg - VIMU_I2C_REG_FIFO_DATA_0)];
    }

    return 0U;
}

void vimu_app_i2c_write_register(uint8_t reg, uint8_t value)
{
    if (reg != VIMU_I2C_REG_COMMAND)
    {
        return;
    }

    vimu_app_queue_i2c_command(value);
}

void I2C_Slave_OnAddressed(uint8_t isTransmitter)
{
    g_vimuI2cRxCount = 0U;
    g_vimuI2cFifoReadActive = 0U;
    g_vimuI2cFifoReadComplete = 0U;
    g_vimuI2cFifoReadCount = 0U;

    if (isTransmitter != 0U)
    {
        g_vimuI2cCurrentRegister = g_vimuI2cRegisterPointer;

        if ((g_vimuI2cCurrentRegister == VIMU_I2C_REG_FIFO_DATA_0) && (g_vimuApp.fifoBlockReady != 0U))
        {
            g_vimuI2cFifoReadActive = 1U;
        }
    }
}

void I2C_Slave_OnRxByte(uint8_t data)
{
    if (g_vimuI2cRxCount == 0U)
    {
        g_vimuI2cRegisterPointer = data;
        g_vimuI2cCurrentRegister = data;
    }
    else
    {
        vimu_app_i2c_write_register(g_vimuI2cCurrentRegister, data);
        g_vimuI2cCurrentRegister++;
        g_vimuI2cRegisterPointer = g_vimuI2cCurrentRegister;
    }

    g_vimuI2cRxCount++;
}

uint8_t I2C_Slave_OnTxByte(void)
{
    uint8_t data;

    data = vimu_app_i2c_read_register(g_vimuI2cCurrentRegister);

    if (g_vimuI2cFifoReadActive != 0U)
    {
        if ((g_vimuI2cCurrentRegister >= VIMU_I2C_REG_FIFO_DATA_0) &&
            (g_vimuI2cCurrentRegister <= VIMU_I2C_REG_FIFO_DATA_LAST))
        {
            g_vimuI2cFifoReadCount++;
            if (g_vimuI2cFifoReadCount >= VIMU_I2C_FIFO_BLOCK_BYTES)
            {
                g_vimuI2cFifoReadActive = 0U;
                g_vimuI2cFifoReadComplete = 1U;
            }
        }
        else
        {
            g_vimuI2cFifoReadActive = 0U;
            g_vimuI2cFifoReadCount = 0U;
        }
    }

    g_vimuI2cCurrentRegister++;
    g_vimuI2cRegisterPointer = g_vimuI2cCurrentRegister;

    return data;
}

void I2C_Slave_OnStop(void)
{
    g_vimuI2cRxCount = 0U;

    if (g_vimuI2cFifoReadComplete != 0U)
    {
        vimu_app_queue_i2c_command(VIMU_I2C_CMD_FIFO_RELEASE);
    }

    g_vimuI2cFifoReadActive = 0U;
    g_vimuI2cFifoReadComplete = 0U;
    g_vimuI2cFifoReadCount = 0U;
}
