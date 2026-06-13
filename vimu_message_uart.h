#ifndef _VIMU_MESSAGE_UART_H_
#define _VIMU_MESSAGE_UART_H_

#include "stdint.h"

/* =========================
 * Protocol constants
 * ========================= */
#define UART_PROTO_START_FRAME_1      0xAA
#define UART_PROTO_START_FRAME_2      0x55
#define LENGTH_DEFAULT                0x00
#define CRC_DEFAULT                   0x0000
#define UART_PROTO_MAX_DATA_LEN       1024U
#define UART_PROTO_HEADER_SIZE        6U
#define UART_PROTO_CRC_SIZE           2U
#define UART_PROTO_FIXED_SIZE         (UART_PROTO_HEADER_SIZE + UART_PROTO_CRC_SIZE)
#define UART_PROTO_MAX_FRAME_LEN      (UART_PROTO_FIXED_SIZE + UART_PROTO_MAX_DATA_LEN)

/* =========================
 * Packet type
 * ========================= */
typedef enum
{
    VIMU_UART_PKT_TYPE_CMD  = 0x01,   // PC -> VIMU
    VIMU_UART_PKT_TYPE_RSP  = 0x02,   // VIMU -> PC
    VIMU_UART_PKT_TYPE_EVT  = 0x03,   // VIMU tự gửi lên PC
    VIMU_UART_PKT_TYPE_ACK  = 0x04,   // ACK ngắn
    VIMU_UART_PKT_TYPE_NACK = 0x05    // Báo lỗi
} uart_pkt_type_t;

/* =========================
 * Command code
 * ========================= */
typedef enum
{
    VIMU_UART_CMD_HANDSHAKE     = 0x10,
    VIMU_UART_CMD_NEW_DATA      = 0x11,
    VIMU_UART_CMD_CLEAR         = 0x12,
    VIMU_UART_CMD_STOP          = 0x13,
    VIMU_UART_CMD_START         = 0x14,
    VIMU_UART_CMD_UPDATE        = 0x15,

    VIMU_UART_CMD_REQ_MORE_DATA = 0x20,
    VIMU_UART_CMD_STATUS_PUSH   = 0x21
} uart_cmd_t;

/* =========================
 * System state
 * ========================= */
typedef enum
{
    VIMU_STATE_STOPPED = 0x00,
    VIMU_STATE_RUNNING = 0x01
} vimu_state_t;

/* =========================
 * Buffer level status
 * ========================= */
typedef enum
{
    VIMU_BUF_EMPTY = 0x00,
    VIMU_BUF_1S    = 0x01,
    VIMU_BUF_2S    = 0x02,
    VIMU_BUF_3S    = 0x03,
    VIMU_BUF_4S    = 0x04,
    VIMU_BUF_FULL  = 0x05 // > 4s
} vimu_buf_level_t;

/* =========================
 * Error code for NACK
 * ========================= */
typedef enum
{
    VIMU_UART_ERR_NONE          = 0x00,
    VIMU_UART_ERR_BAD_CRC       = 0x01,
    VIMU_UART_ERR_BAD_LENGTH    = 0x02,
    VIMU_UART_ERR_UNKNOWN_CMD   = 0x03,
    VIMU_UART_ERR_INVALID_STATE = 0x04,
    VIMU_UART_ERR_BUFFER_FULL   = 0x05,
    VIMU_UART_ERR_BUFFER_EMPTY  = 0x06,
    VIMU_UART_ERR_BAD_FORMAT    = 0x07,
    VIMU_UART_ERR_UNSUPPORTED   = 0x08
} uart_error_t;

/* =========================
 * Status flag bit mask
 * Dùng cho field flags, không nhất thiết phải enum
 * nhưng có thể khai enum để dễ đọc code
 * ========================= */
typedef enum
{
    VIMU_FLAG_BUFFER_LOW = (1U << 0),
    VIMU_FLAG_FIFO_FULL  = (1U << 1),
    VIMU_FLAG_IRQ_ACTIVE = (1U << 2),
    VIMU_FLAG_I2C_ENABLE = (1U << 3),
    VIMU_FLAG_RING_EMPTY = (1U << 4)
} vimu_status_flag_t;

/*
 * Status payload:
 * [0] state, [1] buffer level, [2] flags,
 * [3..4] ring samples LE, [5..6] free ring samples LE.
 */

/* =========================
 * UART parser state machine
 * ========================= */
typedef enum
{
    VIMU_UART_RX_WAIT_START_FRAME = 0,
    VIMU_UART_RX_WAIT_TYPE,
    VIMU_UART_RX_WAIT_CMD,
    VIMU_UART_RX_WAIT_LEN_L,
    VIMU_UART_RX_WAIT_LEN_H,
    VIMU_UART_RX_WAIT_PAYLOAD,
    VIMU_UART_RX_WAIT_CRC_L,
    VIMU_UART_RX_WAIT_CRC_H
} uart_rx_state_t;

typedef struct
{
    uint16_t startFrame;   // 0xAA55
    uint8_t typeMessage;  
    uint8_t cmdCode;
    uint16_t lengthData;
    uint8_t data[UART_PROTO_MAX_DATA_LEN];
    uint16_t crc;
} frame_Message_t;

uint16_t Message_Create_Frame(const frame_Message_t *frameIn, uint8_t *frameOutBytes);
uint8_t Message_Detect_Frame(const uint8_t *frameInBytes, frame_Message_t *frameOut);
uint16_t Check_Sum(const uint8_t *buf, uint16_t len);


#endif /* _VIMU_MESSAGE_UART_H_ */
