#include "vimu_message_uart.h"

static uint16_t Message_Bytes_To_Uint16(uint8_t low_byte, uint8_t high_byte)
{
    return (uint16_t)((uint16_t)low_byte | ((uint16_t)high_byte << 8));
}

uint16_t Message_Create_Frame(frame_Message_t frameIn, uint8_t *frameOutBytes)
{
    uint16_t idx;
    uint16_t crc;

    if (frameOutBytes == 0)
    {
        return 0;
    }

    if (frameIn.lengthData > UART_PROTO_MAX_DATA_LEN)
    {
        return 0;
    }

    frameOutBytes[0] = UART_PROTO_START_FRAME_1;
    frameOutBytes[1] = UART_PROTO_START_FRAME_2;
    frameOutBytes[2] = frameIn.typeMessage;
    frameOutBytes[3] = frameIn.cmdCode;
    frameOutBytes[4] = (uint8_t)(frameIn.lengthData & 0xFFU);
    frameOutBytes[5] = (uint8_t)((frameIn.lengthData >> 8) & 0xFFU);

    for (idx = 0; idx < frameIn.lengthData; idx++)
    {
        frameOutBytes[UART_PROTO_HEADER_SIZE + idx] = frameIn.data[idx];
    }

    crc = Check_Sum(frameOutBytes, (uint16_t)(UART_PROTO_HEADER_SIZE + frameIn.lengthData));

    frameOutBytes[UART_PROTO_HEADER_SIZE + frameIn.lengthData] = (uint8_t)(crc & 0xFFU);
    frameOutBytes[UART_PROTO_HEADER_SIZE + frameIn.lengthData + 1U] = (uint8_t)((crc >> 8) & 0xFFU);

    return (uint16_t)(UART_PROTO_FIXED_SIZE + frameIn.lengthData);
}

uint8_t Message_Detect_Frame(const uint8_t *frameInBytes, frame_Message_t *frameOut)
{
    uint16_t idx;
    uint16_t crc_pos;
    uint16_t crc_rx;
    uint16_t crc_calc;

    if ((frameInBytes == 0) || (frameOut == 0))
    {
        return 0;
    }

    if ((frameInBytes[0] != UART_PROTO_START_FRAME_1) || (frameInBytes[1] != UART_PROTO_START_FRAME_2))
    {
        return 0;
    }

    frameOut->startFrame = Message_Bytes_To_Uint16(frameInBytes[1], frameInBytes[0]);

    frameOut->typeMessage = frameInBytes[2];

    frameOut->cmdCode = frameInBytes[3];

    frameOut->lengthData = Message_Bytes_To_Uint16(frameInBytes[4], frameInBytes[5]);
    
    if (frameOut->lengthData > UART_PROTO_MAX_DATA_LEN)
    {
        return 0;
    }

    for (idx = 0; idx < frameOut->lengthData; idx++)
    {
        frameOut->data[idx] = frameInBytes[UART_PROTO_HEADER_SIZE + idx];
    }

    crc_pos = (uint16_t)(UART_PROTO_HEADER_SIZE + frameOut->lengthData);
    crc_rx = Message_Bytes_To_Uint16(frameInBytes[crc_pos], frameInBytes[crc_pos + 1U]);
    crc_calc = Check_Sum(frameInBytes, crc_pos);

    frameOut->crc = crc_rx;

    if (crc_rx != crc_calc)
    {
        return 0;
    }

    return 1;
}

uint16_t Check_Sum(const uint8_t *buf, uint16_t len)
{
    uint16_t crc;
    uint16_t pos;
    uint8_t bit;

    if (buf == 0)
    {
        return 0;
    }

    crc = 0xFFFF;

    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];

        for (bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}
