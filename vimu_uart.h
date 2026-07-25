#ifndef _VIMU_UART_H_
#define _VIMU_UART_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"                  // Device header
#include "vimu_message_uart.h"
/*
    *************************************************************************
    * @file 	vimu_uart.h
    * @author 	VIMU Team
    * @version V1.0
    * @date 	2024-06-01
    * @brief 	This file contains all the functions prototypes for UART communication.
    *************************************************************************
*/
/*
 * Kept below I2C1 (preempt 0-1) and TIM2 (preempt 2, see vimu_timer.c) so a
 * long UART RX burst can never delay the I2C slave ISR and stretch SCL for
 * the ESP32 RTC-I2C master, which has no bus-recovery of its own.
 */
typedef enum
{
	UART1_Priority = 0x03,
	UART2_Priority,
	UART3_Priority,
	TIM4_Priority

}NVIC_Priority;

typedef struct 
{
	void (*Init)(uint32_t baudrate, uint8_t remap);
	void (*Print)(const char *str, ...);
	uint8_t (*Scan)(uint8_t *data);
	
}UART_TypedefStruct;

extern UART_TypedefStruct UART1;
extern UART_TypedefStruct UART2;
extern UART_TypedefStruct UART3;


void vimu_uart_init(uint32_t baudrate, uint8_t remap);
void vimu_uart_send_bytes(USART_TypeDef *UART, const uint8_t *data, uint16_t length);
uint8_t vimu_uart_receive_frame_bytes(USART_TypeDef *UART,
                                      uint8_t *data,
                                      uint16_t *length,
                                      uint16_t max_length);
void vimu_uart_rx_callback(USART_TypeDef *UART, uint8_t rxByte);


#ifdef __cplusplus
}
#endif
#endif
