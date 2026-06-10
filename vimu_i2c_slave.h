#ifndef VIMU_I2C_SLAVE_H
#define VIMU_I2C_SLAVE_H

#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "vimu_fifo.h"

#define VIMU_I2C_DEFAULT_ADDRESS          0x42U
#define VIMU_I2C_PROTOCOL_VERSION         0x02U

#define VIMU_I2C_IRQ_GPIO_CLK            RCC_APB2Periph_GPIOA
#define VIMU_I2C_IRQ_GPIO_PORT           GPIOA
#define VIMU_I2C_IRQ_GPIO_PIN            GPIO_Pin_8

#define VIMU_I2C_REG_STATE               0x00U
#define VIMU_I2C_REG_BUFFER_LEVEL        0x01U
#define VIMU_I2C_REG_FLAGS               0x02U
#define VIMU_I2C_REG_FIFO_SAMPLE_COUNT   0x03U
#define VIMU_I2C_REG_RING_SAMPLE_COUNT   0x04U
#define VIMU_I2C_REG_TOTAL_SAMPLE_COUNT  0x05U
#define VIMU_I2C_REG_FIFO_BLOCK_READY    0x06U
#define VIMU_I2C_REG_FIFO_BLOCK_BYTES    0x07U
#define VIMU_I2C_REG_PROTOCOL_VERSION    0x08U
#define VIMU_I2C_REG_FIFO_DATA_0         0x40U
#define VIMU_I2C_REG_FIFO_DATA_LAST      0xFFU
#define VIMU_I2C_REG_COMMAND             0x20U

#define VIMU_I2C_FIFO_BLOCK_BYTES        (VIMU_FIFO_SAMPLE_CAPACITY * VIMU_UART_SAMPLE_SIZE)

#define VIMU_I2C_CMD_START               0x01U
#define VIMU_I2C_CMD_STOP                0x02U
#define VIMU_I2C_CMD_CLEAR               0x03U
#define VIMU_I2C_CMD_FIFO_RELEASE        0x04U

void I2C_Slave_Init(uint8_t address);
void I2C_Slave_SetIrq(uint8_t active);

void I2C_Slave_OnAddressed(uint8_t isTransmitter);
void I2C_Slave_OnRxByte(uint8_t data);
uint8_t I2C_Slave_OnTxByte(void);
void I2C_Slave_OnStop(void);

#endif /* VIMU_I2C_SLAVE_H */
