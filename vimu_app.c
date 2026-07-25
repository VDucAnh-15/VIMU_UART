#include "stm32f10x.h"
#include "vimu_app.h"
#include "vimu_i2c_slave.h"
#include "vimu_ring_buffer.h"
#include "vimu_timer.h"
#include "vimu_uart.h"

//DAV 
/* ── App state ──────────────────────────────────────────────────────── */
typedef struct
{
    vimu_ring_buffer_t rxDataRing;
    uint8_t            ringStorage[VIMU_RING_STORAGE_SIZE];
    vimu_fifo_t        fifo;
    volatile vimu_state_t state;
    volatile uint8_t   linkReady;
    volatile uint8_t   requestMoreDataPending;
    volatile uint8_t   requestMoreDataLatched;
    volatile uint8_t   irqSignalActive;
    volatile uint8_t   pendingI2cCommand;
    volatile uint8_t   i2cCommandPending;
    volatile uint16_t  pendingFillTicks;
    uint16_t           lastDataSequence;
    uint8_t            lastDataSequenceValid;
} vimu_uart_app_t;

static vimu_uart_app_t g_vimuApp;

/* ── MMA8451 shadow config registers (written by ESP32 over I2C) ────── */
static uint8_t g_mma_ctrl_reg1    = 0U;
static uint8_t g_mma_xyz_data_cfg = 0U;
static uint8_t g_mma_f_setup      = 0U;
static uint8_t g_mma_ctrl_reg2    = 0U;
static uint8_t g_mma_ctrl_reg3    = MMA8451_CTRL3_IPOL; /* active-high default */
static uint8_t g_mma_ctrl_reg4    = 0U;
static uint8_t g_mma_ctrl_reg5    = 0U;

/* ── I2C register pointer (shared between OnRxByte and OnTxByte) ────── */
static volatile uint8_t g_vimuI2cRegisterPointer = MMA8451_REG_F_STATUS;
static volatile uint8_t g_vimuI2cCurrentRegister = MMA8451_REG_F_STATUS;
static volatile uint8_t g_vimuI2cRxCount         = 0U;

/* ── FIFO streaming state for I2C burst reads (ISR context) ─────────── */
static volatile uint8_t g_i2cFifoSample[VIMU_UART_SAMPLE_SIZE]; /* staged sample */
static volatile uint8_t g_i2cFifoByteIdx    = 0U;  /* byte offset within staged sample */
static volatile uint8_t g_i2cFifoReadActive = 0U;  /* 1 while serving FIFO data */

/* ── Forward declarations ───────────────────────────────────────────── */
static uint8_t  vimu_fifo_transfer_one_from_ring(void);
static uint16_t vimu_app_ring_sample_count(void);
static uint16_t vimu_app_free_sample_count(void);
static vimu_buf_level_t vimu_app_get_buffer_level(uint16_t sampleCount);
static uint8_t  vimu_app_get_status_flags(void);
static void     vimu_app_build_status_payload(uint8_t *payload);
static void     vimu_app_update_low_watermark(void);
static void     vimu_uart_send_frame(uint8_t type, uint8_t cmd, const uint8_t *payload, uint16_t len);
static void     vimu_uart_send_ack(uint8_t cmd);
static void     vimu_uart_send_nack(uint8_t cmd, uart_error_t errorCode);
static void     vimu_uart_send_status_response(uint8_t cmd);
static void     vimu_uart_send_status_push(void);
static void     vimu_uart_send_request_more_data(void);
static void     vimu_handle_frame(const frame_Message_t *rxFrame);
static void     vimu_app_fill_fifo_one_sample(void);
static void     vimu_app_update_fifo_irq(void);
static void     vimu_app_clear_buffers(void);
static void     vimu_app_stop_streaming(void);
static void     vimu_app_start_streaming(void);
static void     vimu_app_queue_i2c_command(uint8_t command);
static void     vimu_app_process_i2c_command(uint8_t command);
static void     vimu_app_set_irq_signal(uint8_t active);
static void     vimu_app_set_i2c_irq_mask(uint8_t masked);

/* ── FIFO helpers ───────────────────────────────────────────────────── */

static uint8_t vimu_fifo_transfer_one_from_ring(void)
{
    uint8_t  sampleBytes[VIMU_UART_SAMPLE_SIZE];
    uint16_t readCount;
    uint8_t  pushed;

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

    vimu_app_set_i2c_irq_mask(1U);
    pushed = vimu_fifo_push(&g_vimuApp.fifo, sampleBytes);
    vimu_app_set_i2c_irq_mask(0U);

    return pushed;
}

static uint16_t vimu_app_ring_sample_count(void)
{
    return (uint16_t)(vimu_ring_buffer_available(&g_vimuApp.rxDataRing) / VIMU_UART_SAMPLE_SIZE);
}

static uint16_t vimu_app_free_sample_count(void)
{
    uint16_t ringSamples = vimu_app_ring_sample_count();
    if (ringSamples >= VIMU_TOTAL_SAMPLE_CAPACITY) return 0U;
    return (uint16_t)(VIMU_TOTAL_SAMPLE_CAPACITY - ringSamples);
}

/* ── Status helpers ─────────────────────────────────────────────────── */

static vimu_buf_level_t vimu_app_get_buffer_level(uint16_t sampleCount)
{
    if (sampleCount == 0U)   return VIMU_BUF_EMPTY;
    if (sampleCount <= VIMU_TIMER_FILL_RATE_HZ)        return VIMU_BUF_1S;
    if (sampleCount <= (VIMU_TIMER_FILL_RATE_HZ * 2U)) return VIMU_BUF_2S;
    if (sampleCount <= (VIMU_TIMER_FILL_RATE_HZ * 3U)) return VIMU_BUF_3S;
    if (sampleCount <= (VIMU_TIMER_FILL_RATE_HZ * 4U)) return VIMU_BUF_4S;
    return VIMU_BUF_FULL;
}

static uint8_t vimu_app_get_status_flags(void)
{
    uint8_t  flags     = 0U;
    uint16_t ringSamples = vimu_app_ring_sample_count();

    if (ringSamples < VIMU_LOW_WATERMARK_SAMPLES)
    {
        flags |= VIMU_FLAG_BUFFER_LOW;
    }

    if (vimu_fifo_is_full(&g_vimuApp.fifo) != 0U)
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
    uint16_t freeSamples;

    if (payload == 0) return;

    ringSamples  = vimu_app_ring_sample_count();
    freeSamples  = vimu_app_free_sample_count();
    payload[0]   = (uint8_t)g_vimuApp.state;
    payload[1]   = (uint8_t)vimu_app_get_buffer_level(ringSamples);
    payload[2]   = vimu_app_get_status_flags();
    payload[3]   = (uint8_t)(ringSamples & 0xFFU);
    payload[4]   = (uint8_t)((ringSamples >> 8) & 0xFFU);
    payload[5]   = (uint8_t)(freeSamples & 0xFFU);
    payload[6]   = (uint8_t)((freeSamples >> 8) & 0xFFU);
    payload[7]   = VIMU_FIRMWARE_PROTOCOL_VERSION;
}

/* ── IRQ control ────────────────────────────────────────────────────── */

static void vimu_app_set_irq_signal(uint8_t active)
{
    g_vimuApp.irqSignalActive = (active != 0U) ? 1U : 0U;
    I2C_Slave_SetIrq(g_vimuApp.irqSignalActive);
}

static void vimu_app_set_i2c_irq_mask(uint8_t masked)
{
    if (masked != 0U)
    {
        NVIC_DisableIRQ(I2C1_EV_IRQn);
        NVIC_DisableIRQ(I2C1_ER_IRQn);
    }
    else
    {
        NVIC_EnableIRQ(I2C1_ER_IRQn);
        NVIC_EnableIRQ(I2C1_EV_IRQn);
    }
}

/*
 * Assert IRQ when: active mode + FIFO mode enabled + INT enabled
 * + FIFO count > WMRK field  (MMA8451: fires when F_CNT > WMRK).
 */
static void vimu_app_update_fifo_irq(void)
{
    uint8_t active     = (g_mma_ctrl_reg1 & MMA8451_CTRL1_ACTIVE) != 0U;
    uint8_t fifoMode   = g_mma_f_setup & MMA8451_F_MODE_MASK;
    uint8_t intEnabled = (g_mma_ctrl_reg4 & MMA8451_INT_EN_FIFO) != 0U;
    uint8_t count      = vimu_fifo_sample_count(&g_vimuApp.fifo);
    uint8_t wmrk       = g_mma_f_setup & MMA8451_F_WMRK_MASK;
    uint8_t assert;

    assert = (active &&
              (fifoMode != MMA8451_F_MODE_DISABLED) &&
              intEnabled &&
              (count > wmrk)) ? 1U : 0U;

    vimu_app_set_irq_signal(assert);
}

/* ── State machine ──────────────────────────────────────────────────── */

static void vimu_app_clear_buffers(void)
{
    vimu_ring_buffer_clear(&g_vimuApp.rxDataRing);
    vimu_fifo_clear(&g_vimuApp.fifo);
    g_vimuApp.pendingFillTicks = 0U;
    vimu_app_update_fifo_irq();
    vimu_app_update_low_watermark();
    vimu_uart_send_status_push();
}

static void vimu_app_stop_streaming(void)
{
    g_vimuApp.state            = VIMU_STATE_STOPPED;
    g_vimuApp.pendingFillTicks = 0U;
    vimu_app_update_fifo_irq();
    vimu_app_update_low_watermark();
}

static void vimu_app_start_streaming(void)
{
    g_vimuApp.state            = VIMU_STATE_RUNNING;
    g_vimuApp.pendingFillTicks = 0U;
    vimu_app_update_fifo_irq();
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
        case VIMU_I2C_CMD_START: vimu_app_start_streaming(); break;
        case VIMU_I2C_CMD_STOP:  vimu_app_stop_streaming();  break;
        case VIMU_I2C_CMD_CLEAR: vimu_app_clear_buffers();   break;
        default: break;
    }
}

/* ── Low-watermark (UART flow control) ──────────────────────────────── */

static void vimu_app_update_low_watermark(void)
{
    uint16_t ringSamples = vimu_app_ring_sample_count();

    if ((g_vimuApp.linkReady != 0U) && (g_vimuApp.state == VIMU_STATE_RUNNING))
    {
        if ((ringSamples < VIMU_LOW_WATERMARK_SAMPLES) &&
            (g_vimuApp.requestMoreDataLatched == 0U))
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

/* ── UART framing ───────────────────────────────────────────────────── */

static void vimu_uart_send_frame(uint8_t type, uint8_t cmd,
                                 const uint8_t *payload, uint16_t payloadLength)
{
    static frame_Message_t txFrame;
    static uint8_t         txBytes[UART_PROTO_MAX_FRAME_LEN];
    uint16_t idx;
    uint16_t txLength;

    txFrame.startFrame   = 0U;
    txFrame.typeMessage  = type;
    txFrame.cmdCode      = cmd;
    txFrame.lengthData   = payloadLength;
    txFrame.crc          = 0U;

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

    if (g_vimuApp.linkReady == 0U) return;

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

/* ── UART command handler ───────────────────────────────────────────── */

static void vimu_handle_frame(const frame_Message_t *rxFrame)
{
    uint16_t sampleCount;
    uint16_t writtenCount;
    uint16_t dataOffset;
    uint16_t dataSequence;

    if (rxFrame == 0) return;

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
            /*
             * Reliable format: [sequence_le16][N * 6-byte samples].  A retry
             * with the same sequence is ACKed without writing twice, which
             * makes a lost ACK safe.  Keep accepting the legacy N*6 format
             * for older tools, but only the sequenced format is idempotent.
             */
            dataOffset = 0U;
            dataSequence = 0U;
            if ((rxFrame->lengthData >= (2U + VIMU_UART_SAMPLE_SIZE)) &&
                (((rxFrame->lengthData - 2U) % VIMU_UART_SAMPLE_SIZE) == 0U))
            {
                dataOffset = 2U;
                dataSequence = (uint16_t)((uint16_t)rxFrame->data[0] |
                                           ((uint16_t)rxFrame->data[1] << 8));
                if ((g_vimuApp.lastDataSequenceValid != 0U) &&
                    (dataSequence == g_vimuApp.lastDataSequence))
                {
                    vimu_uart_send_ack(rxFrame->cmdCode);
                    break;
                }
            }
            else if ((rxFrame->lengthData == 0U) ||
                     ((rxFrame->lengthData % VIMU_UART_SAMPLE_SIZE) != 0U))
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BAD_LENGTH);
                break;
            }
            sampleCount = (uint16_t)((rxFrame->lengthData - dataOffset) /
                                     VIMU_UART_SAMPLE_SIZE);
            if (sampleCount > vimu_app_free_sample_count())
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BUFFER_FULL);
                break;
            }
            writtenCount = vimu_ring_buffer_write(&g_vimuApp.rxDataRing,
                                                  &rxFrame->data[dataOffset],
                                                  (uint16_t)(rxFrame->lengthData - dataOffset));
            if (writtenCount != (uint16_t)(rxFrame->lengthData - dataOffset))
            {
                vimu_uart_send_nack(rxFrame->cmdCode, VIMU_UART_ERR_BUFFER_FULL);
                break;
            }
            if (dataOffset != 0U)
            {
                g_vimuApp.lastDataSequence = dataSequence;
                g_vimuApp.lastDataSequenceValid = 1U;
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

/* ── FIFO fill (called from main loop) ──────────────────────────────── */

static void vimu_app_fill_fifo_one_sample(void)
{
    uint8_t pushed;

    if (g_vimuApp.state != VIMU_STATE_RUNNING) return;

    pushed = vimu_fifo_transfer_one_from_ring();

    if (pushed != 0U)
    {
        vimu_app_update_fifo_irq();
    }

    vimu_app_update_low_watermark();
}

/* ── Public API ─────────────────────────────────────────────────────── */

void vimu_app_init(void)
{
    vimu_ring_buffer_init(&g_vimuApp.rxDataRing,
                          g_vimuApp.ringStorage,
                          VIMU_RING_STORAGE_SIZE);
    vimu_fifo_clear(&g_vimuApp.fifo);

    g_vimuApp.state                   = VIMU_STATE_STOPPED;
    g_vimuApp.linkReady               = 0U;
    g_vimuApp.requestMoreDataPending  = 0U;
    g_vimuApp.requestMoreDataLatched  = 0U;
    g_vimuApp.irqSignalActive         = 0U;
    g_vimuApp.pendingI2cCommand       = 0U;
    g_vimuApp.i2cCommandPending       = 0U;
    g_vimuApp.pendingFillTicks        = 0U;
    g_vimuApp.lastDataSequence        = 0U;
    g_vimuApp.lastDataSequenceValid   = 0U;
}

void vimu_app_process_uart_frame(void)
{
    static uint8_t        rxBytes[UART_PROTO_MAX_FRAME_LEN];
    static frame_Message_t rxFrame;
    uint16_t rxLength = 0U;

    if (vimu_uart_receive_frame_bytes(USART1, rxBytes, &rxLength,
                                       UART_PROTO_MAX_FRAME_LEN) == 0U)
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

void vimu_app_service_runtime(void)
{
    uint16_t pendingTicks;
    uint8_t  i2cCommand;
    uint8_t  hasI2cCommand;

    __disable_irq();
    pendingTicks   = g_vimuApp.pendingFillTicks;
    g_vimuApp.pendingFillTicks = 0U;
    hasI2cCommand  = g_vimuApp.i2cCommandPending;
    i2cCommand     = g_vimuApp.pendingI2cCommand;
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
    /* Independent of streaming state: a wedged I2C bus must be detected and
     * recovered even while stopped, otherwise the next START never works. */
    I2C_Slave_PollBusHealth();

    if (g_vimuApp.state != VIMU_STATE_RUNNING) return;

    if (g_vimuApp.pendingFillTicks < 0xFFFFU)
    {
        g_vimuApp.pendingFillTicks++;
    }
}

/* ── MMA8451 I2C register read ──────────────────────────────────────── */

uint8_t vimu_app_i2c_read_register(uint8_t reg)
{
    uint8_t count;
    uint8_t wmrk;
    uint8_t status;

    switch (reg)
    {
        case MMA8451_REG_F_STATUS:
            count  = vimu_fifo_sample_count(&g_vimuApp.fifo);
            wmrk   = g_mma_f_setup & MMA8451_F_WMRK_MASK;
            status = count & MMA8451_F_CNT_MASK;
            if (count > wmrk)
                status |= MMA8451_F_WMRK_FLAG;
            if (vimu_fifo_is_full(&g_vimuApp.fifo) != 0U)
                status |= MMA8451_F_OVF;
            return status;

        /* OUT registers are served from staging buffer in I2C_Slave_OnTxByte.
         * Return 0 if accessed outside of a FIFO read transaction. */
        case MMA8451_REG_OUT_X_MSB:
        case MMA8451_REG_OUT_X_LSB:
        case MMA8451_REG_OUT_Y_MSB:
        case MMA8451_REG_OUT_Y_LSB:
        case MMA8451_REG_OUT_Z_MSB:
        case MMA8451_REG_OUT_Z_LSB:
            return 0U;

        case MMA8451_REG_F_SETUP:        return g_mma_f_setup;
        case MMA8451_REG_WHO_AM_I:       return MMA8451_WHO_AM_I_VALUE;
        case MMA8451_REG_XYZ_DATA_CFG:   return g_mma_xyz_data_cfg;
        case MMA8451_REG_CTRL_REG1:      return g_mma_ctrl_reg1;
        case MMA8451_REG_CTRL_REG2:      return g_mma_ctrl_reg2;
        case MMA8451_REG_CTRL_REG3:      return g_mma_ctrl_reg3;
        case MMA8451_REG_CTRL_REG4:      return g_mma_ctrl_reg4;
        case MMA8451_REG_CTRL_REG5:      return g_mma_ctrl_reg5;

        default: return 0U;
    }
}

/* ── MMA8451 I2C register write ─────────────────────────────────────── */

void vimu_app_i2c_write_register(uint8_t reg, uint8_t value)
{
    switch (reg)
    {
        case MMA8451_REG_CTRL_REG1:
            g_mma_ctrl_reg1 = value;
            if (value & MMA8451_CTRL1_ACTIVE)
                vimu_app_queue_i2c_command(VIMU_I2C_CMD_START);
            else
                vimu_app_queue_i2c_command(VIMU_I2C_CMD_STOP);
            break;

        case MMA8451_REG_XYZ_DATA_CFG:
            g_mma_xyz_data_cfg = value;
            break;

        case MMA8451_REG_F_SETUP:
            g_mma_f_setup = value;
            break;

        case MMA8451_REG_CTRL_REG2:
            g_mma_ctrl_reg2 = value;
            break;

        case MMA8451_REG_CTRL_REG3:
            g_mma_ctrl_reg3 = value;
            break;

        case MMA8451_REG_CTRL_REG4:
            g_mma_ctrl_reg4 = value;
            break;

        case MMA8451_REG_CTRL_REG5:
            g_mma_ctrl_reg5 = value;
            break;

        default:
            break;
    }
}

/* ── I2C slave callbacks (called from ISR) ──────────────────────────── */

/*
 * Called when the master addresses this slave.
 * isTransmitter=1: master wants to read  → pre-load first FIFO sample.
 * isTransmitter=0: master wants to write → reset Rx counter.
 */
void I2C_Slave_OnAddressed(uint8_t isTransmitter)
{
    g_vimuI2cRxCount      = 0U;
    g_i2cFifoReadActive   = 0U;
    g_i2cFifoByteIdx      = 0U;

    if (isTransmitter != 0U)
    {
        g_vimuI2cCurrentRegister = g_vimuI2cRegisterPointer;

        /* Pre-load first FIFO sample when reading OUT_X_MSB */
        if (g_vimuI2cCurrentRegister == MMA8451_REG_OUT_X_MSB)
        {
            vimu_sample_t sample;
            if (vimu_fifo_pop(&g_vimuApp.fifo, &sample) != 0U)
            {
                /* Convert LE int16 → MMA8451 big-endian per axis */
                g_i2cFifoSample[0] = sample.bytes[1]; /* X_MSB */
                g_i2cFifoSample[1] = sample.bytes[0]; /* X_LSB */
                g_i2cFifoSample[2] = sample.bytes[3]; /* Y_MSB */
                g_i2cFifoSample[3] = sample.bytes[2]; /* Y_LSB */
                g_i2cFifoSample[4] = sample.bytes[5]; /* Z_MSB */
                g_i2cFifoSample[5] = sample.bytes[4]; /* Z_LSB */
                g_i2cFifoByteIdx   = 0U;
                g_i2cFifoReadActive = 1U;
            }
        }
    }
}

/*
 * Called for each byte the master writes after addressing.
 * Byte 0 = register address; byte 1+ = data bytes.
 */
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

/*
 * Called for each byte the master reads.
 *
 * If a FIFO read is active (register pointer was OUT_X_MSB) we serve bytes
 * from the staging buffer and pre-load the next sample every 6 bytes,
 * enabling efficient burst reads of multiple samples in one transaction.
 * All other registers go through the normal register-map handler.
 */
uint8_t I2C_Slave_OnTxByte(void)
{
    uint8_t data = 0U;
    uint8_t reg  = g_vimuI2cCurrentRegister;

    if (g_i2cFifoReadActive != 0U)
    {
        if (g_i2cFifoByteIdx >= VIMU_UART_SAMPLE_SIZE)
        {
            /* One sample fully transmitted – pre-load next if available */
            vimu_sample_t next;
            if (vimu_fifo_pop(&g_vimuApp.fifo, &next) != 0U)
            {
                g_i2cFifoSample[0] = next.bytes[1];
                g_i2cFifoSample[1] = next.bytes[0];
                g_i2cFifoSample[2] = next.bytes[3];
                g_i2cFifoSample[3] = next.bytes[2];
                g_i2cFifoSample[4] = next.bytes[5];
                g_i2cFifoSample[5] = next.bytes[4];
                g_i2cFifoByteIdx = 0U;
            }
            else
            {
                g_i2cFifoReadActive = 0U;
            }
        }

        if (g_i2cFifoReadActive != 0U)
        {
            data = (uint8_t)g_i2cFifoSample[g_i2cFifoByteIdx];
            g_i2cFifoByteIdx++;
        }
    }
    else
    {
        g_i2cFifoReadActive = 0U;
        data = vimu_app_i2c_read_register(reg);
    }

    g_vimuI2cCurrentRegister++;
    g_vimuI2cRegisterPointer = g_vimuI2cCurrentRegister;

    return data;
}

/* Called on STOP condition – update IRQ to reflect current FIFO depth. */
void I2C_Slave_OnStop(void)
{
    g_vimuI2cRxCount      = 0U;
    g_i2cFifoReadActive   = 0U;
    g_i2cFifoByteIdx      = 0U;
    vimu_app_update_fifo_irq();
}

/*
 * Called on BERR/ARLO after the I2C peripheral has been soft-reset.
 * Resets all slave-side state so the next transaction starts clean.
 */
void I2C_Slave_OnError(void)
{
    g_vimuI2cRxCount         = 0U;
    g_vimuI2cRegisterPointer = MMA8451_REG_F_STATUS;
    g_vimuI2cCurrentRegister = MMA8451_REG_F_STATUS;
    g_i2cFifoReadActive      = 0U;
    g_i2cFifoByteIdx         = 0U;
    vimu_app_update_fifo_irq();
}
