#include "vimu_i2c_slave.h"

/*
* Bản đề mô dùng TXE + RXNE với ITBUF
* Hoặc dùng DMA
*/

static uint8_t s_transmitter = 0;

void I2C_Slave_Init(uint8_t address)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);

	I2C_DeInit(I2C1);

	I2C_InitTypeDef i2c;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = (uint16_t)(address << 1);
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2c.I2C_ClockSpeed = 100000;
    I2C_Init(I2C1, &i2c);

	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    I2C_Cmd(I2C1, ENABLE);
}

void I2C1_EV_IRQHandler(void)
{
	uint16_t sr1 = I2C1->SR1;
	
	if (sr1 & I2C_SR1_ADDR)
	{
		uint16_t sr2 = I2C1->SR2;;

		s_transmitter = (sr2 & I2C_SR2_TRA) ? 1u : 0u;
		I2C_Slave_OnAddressed(s_transmitter);
		
		if (s_transmitter)
		{
			I2C1->DR = I2C_Slave_OnTxByte();
		}
		return;
	}

	// Khi RXNE được set sẽ sinh ngắt nếu ITBUF được bật
	if (!s_transmitter && (sr1 & I2C_SR1_RXNE))
	{
		volatile uint8_t data = I2C1->DR;
		I2C_Slave_OnRxByte(data);
		return;
	}

	// Tương tự RXNE
	if (s_transmitter && (sr1 & I2C_SR1_TXE))
	{
		I2C_SendData(I2C1, I2C_Slave_OnTxByte());
		return;
	}

	if (sr1 & I2C_SR1_STOPF)
	{
		I2C1->CR1 = I2C1->CR1; // Ghi lại vào thanh ghi CR1 dderer xóa STOPF
		
		s_transmitter = 0;
		I2C_Slave_OnStop();
		return;
	}
}

void I2C1_ER_IRQHandler(void)
{
	uint16_t sr1 = I2C1->SR1;
	
	if (sr1 & I2C_SR1_AF)
	{
		I2C1->SR1 &= ~I2C_SR1_AF;
		s_transmitter = 0;
	}
	
	if (sr1 & I2C_SR1_OVR)
	{
		uint8_t dummy = I2C1->DR;
		dummy = I2C1->SR1;
		(void)dummy;
	}
	
	if (sr1 & I2C_SR1_BERR)
	{
		I2C1->SR1 &= ~I2C_SR1_BERR;
	}
}
