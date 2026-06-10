#include "vimu_i2c_slave.h"

static uint8_t s_transmitter = 0U;

void I2C_Slave_SetIrq(uint8_t active)
{
    if (active != 0U)
    {
        GPIO_SetBits(VIMU_I2C_IRQ_GPIO_PORT, VIMU_I2C_IRQ_GPIO_PIN);
    }
    else
    {
        GPIO_ResetBits(VIMU_I2C_IRQ_GPIO_PORT, VIMU_I2C_IRQ_GPIO_PIN);
    }
}

void I2C_Slave_Init(uint8_t address)
{
    GPIO_InitTypeDef gpioInit;
    GPIO_InitTypeDef irqGpioInit;
    I2C_InitTypeDef i2cInit;
    NVIC_InitTypeDef nvicInit;

    if ((address == 0U) || (address > 0x7FU))
    {
        return;
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO |
                           VIMU_I2C_IRQ_GPIO_CLK,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    gpioInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpioInit.GPIO_Mode = GPIO_Mode_AF_OD;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpioInit);

    irqGpioInit.GPIO_Pin = VIMU_I2C_IRQ_GPIO_PIN;
    irqGpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    irqGpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(VIMU_I2C_IRQ_GPIO_PORT, &irqGpioInit);
    I2C_Slave_SetIrq(0U);

    I2C_DeInit(I2C1);
    I2C_StructInit(&i2cInit);
    i2cInit.I2C_Mode = I2C_Mode_I2C;
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;
    i2cInit.I2C_OwnAddress1 = (uint16_t)(address << 1);
    i2cInit.I2C_Ack = I2C_Ack_Enable;
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    i2cInit.I2C_ClockSpeed = 100000U;
    I2C_Init(I2C1, &i2cInit);

    nvicInit.NVIC_IRQChannel = I2C1_EV_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1U;
    nvicInit.NVIC_IRQChannelSubPriority = 0U;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);

    nvicInit.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_Init(&nvicInit);

    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE);
    I2C_Cmd(I2C1, ENABLE);
}

void I2C1_EV_IRQHandler(void)
{
    uint16_t sr1;

    sr1 = I2C1->SR1;

    if ((sr1 & I2C_SR1_ADDR) != 0U)
    {
        uint16_t sr2;

        sr2 = I2C1->SR2;
        s_transmitter = ((sr2 & I2C_SR2_TRA) != 0U) ? 1U : 0U;
        I2C_Slave_OnAddressed(s_transmitter);

        if (s_transmitter != 0U)
        {
            I2C1->DR = I2C_Slave_OnTxByte();
        }
        return;
    }

    if ((s_transmitter == 0U) && ((sr1 & I2C_SR1_RXNE) != 0U))
    {
        uint8_t data;

        data = (uint8_t)I2C1->DR;
        I2C_Slave_OnRxByte(data);
        return;
    }

    if ((s_transmitter != 0U) && ((sr1 & I2C_SR1_TXE) != 0U))
    {
        I2C_SendData(I2C1, I2C_Slave_OnTxByte());
        return;
    }

    if ((sr1 & I2C_SR1_STOPF) != 0U)
    {
        I2C1->CR1 = (uint16_t)(I2C1->CR1 | I2C_CR1_PE);
        s_transmitter = 0U;
        I2C_Slave_OnStop();
    }
}

void I2C1_ER_IRQHandler(void)
{
    uint16_t sr1;

    sr1 = I2C1->SR1;

    if ((sr1 & I2C_SR1_AF) != 0U)
    {
        I2C1->SR1 &= (uint16_t)(~I2C_SR1_AF);
        s_transmitter = 0U;
    }

    if ((sr1 & I2C_SR1_OVR) != 0U)
    {
        volatile uint8_t dummyData;
        volatile uint16_t dummyStatus;

        dummyData = (uint8_t)I2C1->DR;
        dummyStatus = I2C1->SR1;
        (void)dummyData;
        (void)dummyStatus;
    }

    if ((sr1 & I2C_SR1_BERR) != 0U)
    {
        I2C1->SR1 &= (uint16_t)(~I2C_SR1_BERR);
    }
}
