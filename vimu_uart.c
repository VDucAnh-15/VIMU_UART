#include "vimu_uart.h"
#include "vimu_timer.h"
#include <stdio.h>
#include <stdarg.h>

#define UART_FRAME_GAP_TIMEOUT_TICKS 5U

UART_TypedefStruct UART1;
UART_TypedefStruct UART2;
UART_TypedefStruct UART3;

typedef enum 
{
	U1 = 0x00,
	U2, 
	U3
	
}UARTx_e;

typedef struct
{
	uart_rx_state_t state;
	uint8_t startMatched;
	uint16_t lengthData;
	uint16_t payloadIndex;
	uint32_t lastRxTick;
	uint8_t frameBytes[UART_PROTO_MAX_FRAME_LEN];
} UART_FrameReceiver_t;

volatile uint8_t UARTx_dataReceived[3][UART_PROTO_MAX_FRAME_LEN + 1U] = {0};
volatile uint8_t UARTx_rx_flag[3] = {0,0,0};
volatile uint16_t UARTx_id[3]	= {0,0,0};
volatile uint8_t UARTx_timeout[3] = {0,0,0};
static UART_FrameReceiver_t UARTx_frameReceiver[3];
static volatile uint8_t UARTx_frameReady[3] = {0, 0, 0};
static volatile uint16_t UARTx_frameLength[3] = {0, 0, 0};
static uint8_t UARTx_frameReceived[3][UART_PROTO_MAX_FRAME_LEN];
char number[50];

void UARTx_Init(USART_TypeDef *UART, uint32_t baudrate, uint8_t remap);
void UARTx_SendData(USART_TypeDef *UART, const char *str, va_list args);
void UARTx_SendBytes(USART_TypeDef *UART, const uint8_t *data, uint16_t length);
uint8_t UARTx_ReceiveData(UARTx_e Ux, uint8_t *data);
static uint8_t UARTx_GetIndex(USART_TypeDef *UART, UARTx_e *Ux);
static void UARTx_ResetFrameReceiver(UARTx_e Ux);
static void UARTx_ClearCompletedFrame(UARTx_e Ux);
static void UARTx_StoreCompletedFrame(UARTx_e Ux);
static void UARTx_PushFrameByte(UARTx_e Ux, uint8_t rxByte);
static void UARTx_HandleRxInterrupt(UARTx_e Ux, USART_TypeDef *UART);

void UART1_Init(uint32_t baudrate, uint8_t remap);
void UART2_Init(uint32_t baudrate, uint8_t remap);
void UART3_Init(uint32_t baudrate, uint8_t remap);
void UART1_Print(const char *str, ...);
void UART2_Print(const char *str, ...);
void UART3_Print(const char *str, ...);
uint8_t UART1_Scan(uint8_t *data);
uint8_t UART2_Scan(uint8_t *data);
uint8_t UART3_Scan(uint8_t *data);


void vimu_uart_init(uint32_t baudrate, uint8_t remap)
{
    UART1_Init(baudrate, remap);
}

void vimu_uart_send_bytes(USART_TypeDef *UART, const uint8_t *data, uint16_t length)
{
	UARTx_SendBytes(UART, data, length);
}

__weak void vimu_uart_rx_callback(USART_TypeDef *UART, uint8_t rxByte)
{
	(void)UART;
	(void)rxByte;
}

uint8_t vimu_uart_receive_frame_bytes(USART_TypeDef *UART,
									  uint8_t *data,
									  uint16_t *length,
									  uint16_t max_length)
{
	UARTx_e Ux;
	uint16_t idx;

	if ((data == 0) || (length == 0) || (max_length == 0U))
	{
		return 0U;
	}

	if (UARTx_GetIndex(UART, &Ux) == 0U)
	{
		return 0U;
	}

	if (UARTx_frameReady[Ux] == 0U)
	{
		return 0U;
	}

	if (UARTx_frameLength[Ux] > max_length)
	{
		UARTx_ClearCompletedFrame(Ux);
		*length = 0U;
		return 0U;
	}

	for (idx = 0U; idx < UARTx_frameLength[Ux]; idx++)
	{
		data[idx] = UARTx_frameReceived[Ux][idx];
	}

	*length = UARTx_frameLength[Ux];
	UARTx_ClearCompletedFrame(Ux);

	return 1U;
}


/**
	*************************************************************************
	* @brief 	Initialize Callback Functions
	* @param 	None
	* @retval None
	*************************************************************************
*/

void UART_FirstInit(void) __attribute__ ((constructor));			// Ham nay se duoc chay truoc khi vao ham main

void UART_FirstInit(void)
{
	UART1.Init  = UART1_Init;
	UART1.Scan  = UART1_Scan;
	UART1.Print = UART1_Print;
	
	UART2.Init  = UART2_Init;
	UART2.Scan  = UART2_Scan;
	UART2.Print = UART2_Print;
	
	UART3.Init  = UART3_Init;
	UART3.Scan  = UART3_Scan;
	UART3.Print = UART3_Print;

	UARTx_ResetFrameReceiver(U1);
	UARTx_ResetFrameReceiver(U2);
	UARTx_ResetFrameReceiver(U3);
	UARTx_ClearCompletedFrame(U1);
	UARTx_ClearCompletedFrame(U2);
	UARTx_ClearCompletedFrame(U3);
}

static uint8_t UARTx_GetIndex(USART_TypeDef *UART, UARTx_e *Ux)
{
	if (Ux == 0)
	{
		return 0U;
	}

	if (UART == USART1)
	{
		*Ux = U1;
		return 1U;
	}

	if (UART == USART2)
	{
		*Ux = U2;
		return 1U;
	}

	if (UART == USART3)
	{
		*Ux = U3;
		return 1U;
	}

	return 0U;
}

static void UARTx_ResetFrameReceiver(UARTx_e Ux)
{
	UARTx_frameReceiver[Ux].state = VIMU_UART_RX_WAIT_START_FRAME;
	UARTx_frameReceiver[Ux].startMatched = 0U;
	UARTx_frameReceiver[Ux].lengthData = 0U;
	UARTx_frameReceiver[Ux].payloadIndex = 0U;
	UARTx_frameReceiver[Ux].lastRxTick = vimu_timer_get_tick();
}

static void UARTx_ClearCompletedFrame(UARTx_e Ux)
{
	UARTx_frameReady[Ux] = 0U;
	UARTx_frameLength[Ux] = 0U;
}

static void UARTx_StoreCompletedFrame(UARTx_e Ux)
{
	uint16_t idx;
	uint16_t totalLength;

	if (UARTx_frameReady[Ux] != 0U)
	{
		return;
	}

	totalLength = (uint16_t)(UART_PROTO_FIXED_SIZE + UARTx_frameReceiver[Ux].lengthData);

	for (idx = 0U; idx < totalLength; idx++)
	{
		UARTx_frameReceived[Ux][idx] = UARTx_frameReceiver[Ux].frameBytes[idx];
	}

	UARTx_frameLength[Ux] = totalLength;
	UARTx_frameReady[Ux] = 1U;
}

static void UARTx_PushFrameByte(UARTx_e Ux, uint8_t rxByte)
{
	UART_FrameReceiver_t *receiver;
	uint32_t nowTick;

	receiver = &UARTx_frameReceiver[Ux];
	nowTick = vimu_timer_get_tick();

	if (((receiver->state != VIMU_UART_RX_WAIT_START_FRAME) ||
		 (receiver->startMatched != 0U)) &&
		((uint32_t)(nowTick - receiver->lastRxTick) > UART_FRAME_GAP_TIMEOUT_TICKS))
	{
		UARTx_ResetFrameReceiver(Ux);
	}

	receiver = &UARTx_frameReceiver[Ux];
	receiver->lastRxTick = nowTick;

	switch (receiver->state)
	{
		case VIMU_UART_RX_WAIT_START_FRAME:
		{
			if (receiver->startMatched == 0U)
			{
				if (rxByte == UART_PROTO_START_FRAME_1)
				{
					receiver->frameBytes[0] = rxByte;
					receiver->startMatched = 1U;
				}
			}
			else
			{
				if (rxByte == UART_PROTO_START_FRAME_2)
				{
					receiver->frameBytes[1] = rxByte;
					receiver->startMatched = 0U;
					receiver->state = VIMU_UART_RX_WAIT_TYPE;
				}
				else if (rxByte == UART_PROTO_START_FRAME_1)
				{
					receiver->frameBytes[0] = rxByte;
					receiver->startMatched = 1U;
				}
				else
				{
					receiver->startMatched = 0U;
				}
			}
			break;
		}

		case VIMU_UART_RX_WAIT_TYPE:
			receiver->frameBytes[2] = rxByte;
			receiver->state = VIMU_UART_RX_WAIT_CMD;
			break;

		case VIMU_UART_RX_WAIT_CMD:
			receiver->frameBytes[3] = rxByte;
			receiver->state = VIMU_UART_RX_WAIT_LEN_L;
			break;

		case VIMU_UART_RX_WAIT_LEN_L:
			receiver->frameBytes[4] = rxByte;
			receiver->lengthData = rxByte;
			receiver->state = VIMU_UART_RX_WAIT_LEN_H;
			break;

		case VIMU_UART_RX_WAIT_LEN_H:
			receiver->frameBytes[5] = rxByte;
			receiver->lengthData |= (uint16_t)((uint16_t)rxByte << 8);

			if (receiver->lengthData > UART_PROTO_MAX_DATA_LEN)
			{
				UARTx_ResetFrameReceiver(Ux);
			}
			else if (receiver->lengthData == 0U)
			{
				receiver->state = VIMU_UART_RX_WAIT_CRC_L;
			}
			else
			{
				receiver->payloadIndex = 0U;
				receiver->state = VIMU_UART_RX_WAIT_PAYLOAD;
			}
			break;

		case VIMU_UART_RX_WAIT_PAYLOAD:
			receiver->frameBytes[UART_PROTO_HEADER_SIZE + receiver->payloadIndex] = rxByte;
			receiver->payloadIndex++;

			if (receiver->payloadIndex >= receiver->lengthData)
			{
				receiver->state = VIMU_UART_RX_WAIT_CRC_L;
			}
			break;

		case VIMU_UART_RX_WAIT_CRC_L:
			receiver->frameBytes[UART_PROTO_HEADER_SIZE + receiver->lengthData] = rxByte;
			receiver->state = VIMU_UART_RX_WAIT_CRC_H;
			break;

		case VIMU_UART_RX_WAIT_CRC_H:
			receiver->frameBytes[UART_PROTO_HEADER_SIZE + receiver->lengthData + 1U] = rxByte;
			UARTx_StoreCompletedFrame(Ux);
			UARTx_ResetFrameReceiver(Ux);
			break;

		default:
			UARTx_ResetFrameReceiver(Ux);
			break;
	}
}

static void UARTx_HandleRxInterrupt(UARTx_e Ux, USART_TypeDef *UART)
{
	uint16_t status;
	uint8_t rxByte;

	/*
	 * RXNEIE also raises the USART IRQ for ORE.  The old handler returned when
	 * RXNE was clear, leaving ORE asserted forever.  Because USART1 has the
	 * highest priority this became an interrupt storm: the main loop, replies
	 * and even a new HANDSHAKE all stopped permanently.
	 *
	 * Read SR followed by DR to clear PE/FE/NE/ORE as required by STM32F1.  A
	 * byte received with any of these errors is discarded and the frame parser
	 * is reset; the PC can then retry the complete, CRC-protected frame.
	 */
	status = UART->SR;
	if ((status & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U)
	{
		rxByte = (uint8_t)UART->DR;
		(void)rxByte;
		UARTx_id[Ux] = 0U;
		UARTx_rx_flag[Ux] = 0U;
		UARTx_timeout[Ux] = 0U;
		UARTx_ResetFrameReceiver(Ux);
		return;
	}

	if ((status & USART_SR_RXNE) == 0U) return;

	rxByte = (uint8_t)UART->DR;

	UARTx_timeout[Ux] = 0;
	if (UARTx_rx_flag[Ux] == 0U)
	{
		UARTx_rx_flag[Ux] = 1U;
	}

	vimu_uart_rx_callback(UART, rxByte);

	if (Ux == U1)
	{
		/* USART1 is the framed VIMU link.  Do not also copy every byte into
		 * the legacy text-scan buffer; that duplicate ISR work increases the
		 * chance of overrun during long unattended transfers. */
		UARTx_PushFrameByte(Ux, rxByte);
	}
	else if (UARTx_id[Ux] < UART_PROTO_MAX_FRAME_LEN)
	{
		UARTx_dataReceived[Ux][UARTx_id[Ux]++] = rxByte;
	}
	else
	{
		UARTx_id[Ux] = 0U;
		UARTx_rx_flag[Ux] = 0U;
	}
}

/**
	*************************************************************************
	* @brief 	UARTx Initialization
	* @param 	USART_TypeDef
	* @param  baudrate
	* @param  remap: remap or not
	* @retval None
	*************************************************************************
*/

void UARTx_Init(USART_TypeDef *UART, uint32_t baudrate, uint8_t remap)
{
	GPIO_InitTypeDef 		GPIO_InitStruct;
	USART_InitTypeDef 	UART_InitStruct;
	
	if(UART == USART1)
	{	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		
		if(remap)
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
			
			GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
			
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
			GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
			
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
			GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7;
			GPIO_Init(GPIOB, &GPIO_InitStruct);
		}
		else
		{
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
			GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;
			GPIO_Init(GPIOA, &GPIO_InitStruct);
			
			GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
			GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
			GPIO_Init(GPIOA, &GPIO_InitStruct);
		}
		
		NVIC_SetPriority(USART1_IRQn, UART1_Priority);
		NVIC_EnableIRQ(USART1_IRQn);
	}
	
	else if(UART == USART2)
	{	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_3;
		GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		NVIC_SetPriority(USART2_IRQn, UART2_Priority);
		NVIC_EnableIRQ(USART2_IRQn);
	}
	
	else if(UART == USART3)
	{	
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
		GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
		GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_11;
		GPIO_Init(GPIOB, &GPIO_InitStruct);
		
		NVIC_SetPriority(USART3_IRQn, UART3_Priority);
		NVIC_EnableIRQ(USART3_IRQn);
	}
	
	UART_InitStruct.USART_BaudRate = baudrate;											/* toc do truyen: baud_rates */
	UART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	UART_InitStruct.USART_Parity = USART_Parity_No;
	UART_InitStruct.USART_StopBits = USART_StopBits_1;
	UART_InitStruct.USART_WordLength = USART_WordLength_8b;
	UART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(UART, &UART_InitStruct);
	
	USART_Cmd(UART,ENABLE);																/* cho phep USART1 hoat dong */
	
	USART_ITConfig(UART,USART_IT_RXNE,ENABLE);						/* ngat nhan USART1 					*/
}

/**
	*************************************************************************
	* @brief 	UARTx Initialization
	* @param  baudrate
	* @param  remap: remap or not
	* @retval None
	*************************************************************************
*/

void UART1_Init(uint32_t baudrate, uint8_t remap)
{
	UARTx_Init(USART1, baudrate, remap);
}

void UART2_Init(uint32_t baudrate, uint8_t remap)
{
	UARTx_Init(USART2, baudrate, remap);
}

void UART3_Init(uint32_t baudrate, uint8_t remap)
{
	UARTx_Init(USART3, baudrate, remap);
}

/**
	*************************************************************************
	* @brief 	UART Print
	* @param  str: format string
	* @param  ...: arguments
	* @retval None
	*************************************************************************
*/

void UART1_Print(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	UARTx_SendData(USART1, str, args);
	va_end(args);
}

void UART2_Print(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	UARTx_SendData(USART2, str, args);
	va_end(args);
}

void UART3_Print(const char *str, ...)
{
	va_list args;
	va_start(args, str);
	UARTx_SendData(USART3, str, args);
	va_end(args);
}

/**
	*************************************************************************
	* @brief 	UART Scan
	* @param  data: received string
	* @retval true if there is a new string
	*************************************************************************
*/

uint8_t UART1_Scan(uint8_t *data)
{
	return UARTx_ReceiveData(U1, data);
}

uint8_t UART2_Scan(uint8_t *data)
{
	return UARTx_ReceiveData(U2, data);
}

uint8_t UART3_Scan(uint8_t *data)
{
	return UARTx_ReceiveData(U3, data);
}

void UARTx_SendData(USART_TypeDef *UART, const char *str, va_list args)
{
	do{
		if(*str == '%')
		{
			switch(*(++str))
			{
				case 's':
				{
					char *temp_str = va_arg(args, char*);
					do
					{
						while(USART_GetFlagStatus(UART,USART_FLAG_TXE)==RESET);
						USART_SendData(UART, *temp_str);
					}while(*(++temp_str));
					
					continue;
				}
				case 'd':
				{
					char *ptr;
					int temp_num = va_arg(args, int);
					sprintf(number, "%d", temp_num);

					ptr = number;

					do {
							while (USART_GetFlagStatus(UART, USART_FLAG_TXE) == RESET);
							USART_SendData(UART, *ptr);
					} while (*(++ptr));
					
					continue;
				}
				case 'f':
				{
					char *ptr;
					float temp_num = va_arg(args, double);
					sprintf(number, "%f", temp_num);
					ptr = number;
					do
					{
						while(USART_GetFlagStatus(UART,USART_FLAG_TXE)==RESET);
						USART_SendData(UART, *ptr);
					}while(*(++ptr));
					
					continue;
				}
			}
		}
		
		while(USART_GetFlagStatus(UART,USART_FLAG_TXE)==RESET);
		USART_SendData(UART, *str);
	}while(*(++str));
}

void UARTx_SendBytes(USART_TypeDef *UART, const uint8_t *data, uint16_t length)
{
	uint16_t idx;

	if ((UART == 0) || (data == 0) || (length == 0U))
	{
		return;
	}

	for (idx = 0; idx < length; idx++)
	{
		while (USART_GetFlagStatus(UART, USART_FLAG_TXE) == RESET);
		USART_SendData(UART, data[idx]);
	}

	while (USART_GetFlagStatus(UART, USART_FLAG_TC) == RESET);
}

uint8_t UARTx_ReceiveData(UARTx_e Ux, uint8_t *data)
{
	uint8_t new_data = 0;
	uint8_t index = 0;
	if(UARTx_rx_flag[Ux] == 0) UARTx_timeout[Ux] = 0;
	if(UARTx_rx_flag[Ux] && UARTx_timeout[Ux] < 1) {
		volatile uint16_t i;
		for(i = 0; i< 0xffff; i++);
		
		UARTx_timeout[Ux]++;
	}
	else if(UARTx_rx_flag[Ux] == 1 && UARTx_timeout[Ux] >= 1)
	{
		UARTx_dataReceived[Ux][UARTx_id[Ux]++] = '\0';
		UARTx_id[Ux] = 0;
		UARTx_rx_flag[Ux] = 0;
		new_data = 1;
		do{ 
			data[index] = UARTx_dataReceived[Ux][index]; 
		}while(UARTx_dataReceived[Ux][index++]);
	}
	return new_data;
}

/**
	*************************************************************************
	* @brief 	USARTxIRQn
	* @param 	None
	* @retval None
	*************************************************************************
*/

void USART1_IRQHandler(void)
{
	UARTx_HandleRxInterrupt(U1, USART1);
}
void USART2_IRQHandler(void)
{
	UARTx_HandleRxInterrupt(U2, USART2);
}
void USART3_IRQHandler(void)
{
	UARTx_HandleRxInterrupt(U3, USART3);
}
