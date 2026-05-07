#include "stm32f10x.h"                  // Device header
#include "vimu_uart.h"
#include "vimu_ring_buffer.h"

#define APP_UART_TEST_BUFFER_SIZE 512U

static vimu_ring_buffer_t g_uartRxRing;
static uint8_t g_uartRxStorage[APP_UART_TEST_BUFFER_SIZE];

void vimu_uart_rx_callback(USART_TypeDef *UART, uint8_t rxByte)
{
	if (UART == USART1)
	{
		(void)vimu_ring_buffer_put(&g_uartRxRing, rxByte);
	}
}

int main(void)
{
	uint8_t txBuffer[64];
	uint16_t txLength;

	vimu_ring_buffer_init(&g_uartRxRing, g_uartRxStorage, APP_UART_TEST_BUFFER_SIZE);
	vimu_uart_init(115200U, 0U);

	while (1)
	{
		txLength = 0U;

		while ((txLength < sizeof(txBuffer)) &&
			   (vimu_ring_buffer_get(&g_uartRxRing, &txBuffer[txLength]) != 0U))
		{
			txLength++;
		}

		if (txLength != 0U)
		{
			vimu_uart_send_bytes(USART1, txBuffer, txLength);
		}
	}
}
