#include "stm32f10x.h"                  // Device header
#include "vimu_uart.h"
#include "vimu_message_uart.h"

static void VIMU_SendFrame(uint8_t typeMessage,
						   uint8_t cmdCode,
						   const uint8_t *payload,
						   uint16_t payloadLength)
{
	frame_Message_t txFrame;
	uint8_t txBytes[UART_PROTO_MAX_FRAME_LEN];
	uint16_t txLength;
	uint16_t idx;

	txFrame.startFrame = 0xAA55U;
	txFrame.typeMessage = typeMessage;
	txFrame.cmdCode = cmdCode;
	txFrame.lengthData = payloadLength;
	txFrame.crc = CRC_DEFAULT;

	for (idx = 0U; idx < payloadLength; idx++)
	{
		txFrame.data[idx] = payload[idx];
	}

	txLength = Message_Create_Frame(txFrame, txBytes);
	if (txLength > 0U)
	{
		vimu_uart_send_bytes(USART1, txBytes, txLength);
	}
}

static void VIMU_SendResponse(const frame_Message_t *rxFrame)
{
	VIMU_SendFrame(VIMU_UART_PKT_TYPE_RSP,
				   rxFrame->cmdCode,
				   rxFrame->data,
				   rxFrame->lengthData);
}

static void VIMU_SendNack(uint8_t cmdCode, uart_error_t errorCode)
{
	uint8_t payload[1];

	payload[0] = (uint8_t)errorCode;
	VIMU_SendFrame(VIMU_UART_PKT_TYPE_NACK, cmdCode, payload, 1U);
}

int main()
{
	uint8_t rxBytes[UART_PROTO_MAX_FRAME_LEN];
	uint16_t rxLength;
	frame_Message_t rxFrame;

	vimu_uart_init(115200U, 0U);

	while(1)
	{
		rxLength = 0U;

		if (vimu_uart_receive_frame_bytes(USART1, rxBytes, &rxLength, UART_PROTO_MAX_FRAME_LEN) != 0U)
		{
			if (Message_Detect_Frame(rxBytes, &rxFrame) != 0U)
			{
				VIMU_SendResponse(&rxFrame);
			}
			else
			{
				VIMU_SendNack(rxBytes[3], VIMU_UART_ERR_BAD_CRC);
			}
		}
	}
}
