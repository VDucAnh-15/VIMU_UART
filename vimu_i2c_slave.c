#include "vimu_i2c_slave.h"

static uint8_t s_transmitter = 0U;
static volatile uint8_t s_recovery_pending = 0U;
static uint8_t s_slave_address = VIMU_I2C_DEFAULT_ADDRESS;

/*
 * Bus-stall watchdog. Incremented once per fill tick (20 ms @ 50 Hz) by
 * I2C_Slave_PollBusHealth() whenever I2C1 is BUSY; reset to 0 by any I2C1
 * event or error ISR activity (real forward progress). A master that
 * times out mid-transaction and vanishes without a STOP never raises AF,
 * BERR or ARLO on this slave, so this is the only way to detect it.
 */
static volatile uint16_t s_bus_stall_ticks = 0U;
#define I2C_BUS_STALL_TICKS_LIMIT   25U   /* ~500 ms @ 50 Hz fill tick */

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

    s_slave_address = address;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO |
                           VIMU_I2C_IRQ_GPIO_CLK,
                           ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    gpioInit.GPIO_Pin = VIMU_I2C_SCL_GPIO_PIN | VIMU_I2C_SDA_GPIO_PIN;
    gpioInit.GPIO_Mode = GPIO_Mode_AF_OD;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(VIMU_I2C_GPIO_PORT, &gpioInit);

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

    /*
     * I2C must preempt everything else in this system. It is the only
     * peripheral with a hard real-time constraint: every microsecond its ISR
     * is delayed is a microsecond SCL is stretched low by this slave. TIM2
     * (fill tick) and USART1 (UART link) can tolerate being deferred by tens
     * of microseconds; the I2C master (ESP32 ULP RTC I2C) cannot - it times
     * out and abandons the transaction without releasing the bus, which was
     * previously wedging the link for good after ~1 hour of runtime. See
     * vimu_timer.c and vimu_uart.h for the matching lower priorities.
     */
    nvicInit.NVIC_IRQChannel = I2C1_EV_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 1U;
    nvicInit.NVIC_IRQChannelSubPriority = 0U;
    nvicInit.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicInit);

    /* Error IRQ must preempt EV: AF and TXE can be set together when the
     * master NACKs the final byte of a slave-transmitter burst. */
    nvicInit.NVIC_IRQChannel = I2C1_ER_IRQn;
    nvicInit.NVIC_IRQChannelPreemptionPriority = 0U;
    NVIC_Init(&nvicInit);

    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE);
    I2C_Cmd(I2C1, ENABLE);
}

/* Crude ~5 us busy-wait at 72 MHz. I2C bus recovery timing is not critical
 * (anything well under the ESP32 RTC-I2C standard-mode period is fine), so
 * a calibrated NOP loop is sufficient - no timer peripheral is spent on it. */
static void i2c_recovery_delay(void)
{
    volatile uint32_t i;

    for (i = 0U; i < 180U; i++)
    {
        __NOP();
    }
}

/*
 * Force the bus idle by bit-banging up to 9 SCL pulses followed by a manual
 * STOP, directly on the GPIO pins. This recovers from the case this slave
 * caused: the ESP32 master timed out and abandoned a read transaction while
 * this MCU was mid-byte as slave-transmitter, so SDA is left driven low by
 * this MCU (or is stuck low from a wedged external device) with no STOP ever
 * seen. The I2C peripheral itself has no way to break out of that - only the
 * pins can. Safe to run even when the bus is already idle.
 */
static void i2c_bus_force_release(void)
{
    GPIO_InitTypeDef gpioInit;
    uint8_t i;

    gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;

    gpioInit.GPIO_Pin = VIMU_I2C_SCL_GPIO_PIN;
    GPIO_Init(VIMU_I2C_GPIO_PORT, &gpioInit);
    gpioInit.GPIO_Pin = VIMU_I2C_SDA_GPIO_PIN;
    GPIO_Init(VIMU_I2C_GPIO_PORT, &gpioInit);

    GPIO_SetBits(VIMU_I2C_GPIO_PORT, VIMU_I2C_SCL_GPIO_PIN | VIMU_I2C_SDA_GPIO_PIN);
    i2c_recovery_delay();

    for (i = 0U; i < 9U; i++)
    {
        if (GPIO_ReadInputDataBit(VIMU_I2C_GPIO_PORT, VIMU_I2C_SDA_GPIO_PIN) == Bit_SET)
        {
            break; /* SDA already released */
        }
        GPIO_ResetBits(VIMU_I2C_GPIO_PORT, VIMU_I2C_SCL_GPIO_PIN);
        i2c_recovery_delay();
        GPIO_SetBits(VIMU_I2C_GPIO_PORT, VIMU_I2C_SCL_GPIO_PIN);
        i2c_recovery_delay();
    }

    /* Manual STOP: SDA rises while SCL is high. */
    GPIO_ResetBits(VIMU_I2C_GPIO_PORT, VIMU_I2C_SDA_GPIO_PIN);
    i2c_recovery_delay();
    GPIO_SetBits(VIMU_I2C_GPIO_PORT, VIMU_I2C_SCL_GPIO_PIN);
    i2c_recovery_delay();
    GPIO_SetBits(VIMU_I2C_GPIO_PORT, VIMU_I2C_SDA_GPIO_PIN);
    i2c_recovery_delay();
}

void I2C_Slave_PollBusHealth(void)
{
    if ((I2C1->SR2 & I2C_SR2_BUSY) == 0U)
    {
        s_bus_stall_ticks = 0U;
        return;
    }

    s_bus_stall_ticks++;
    if (s_bus_stall_ticks < I2C_BUS_STALL_TICKS_LIMIT)
    {
        return;
    }

    s_bus_stall_ticks = 0U;

    /* Mirror the fatal-error path in I2C1_ER_IRQHandler: stop further I2C
     * IRQs until I2C_Slave_Service() performs the full recovery below. */
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);
    s_recovery_pending = 1U;
}

void I2C_Slave_Service(void)
{
    uint8_t recover;

    __disable_irq();
    recover = s_recovery_pending;
    s_recovery_pending = 0U;
    __enable_irq();

    if (recover == 0U) return;

    /* Recover outside ISR: reset the peripheral and restore all slave/NVIC
     * configuration. This also clears a BUSY state left by an interrupted
     * ESP32 transaction. */
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);
    I2C_Cmd(I2C1, DISABLE);

    /* Unwedge the pins first: a peripheral reset alone cannot force an
     * externally-stuck SDA line high, and BUSY can re-latch immediately
     * after I2C_Cmd(ENABLE) if the line is still held low. */
    i2c_bus_force_release();

    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    I2C_Slave_Init(s_slave_address);
}

void I2C1_EV_IRQHandler(void)
{
    uint16_t sr1;

    s_bus_stall_ticks = 0U; /* any EV activity is forward progress */

    sr1 = I2C1->SR1;

    /* Handle the normal end-of-read NACK before TXE. If TXE wins, the event
     * ISR can retrigger continuously and starve both main and the watchdog. */
    if ((sr1 & I2C_SR1_AF) != 0U)
    {
        I2C1->SR1 &= (uint16_t)(~I2C_SR1_AF);
        s_transmitter = 0U;
        I2C_Slave_OnStop();
        return;
    }

    /* STOPF must win over TXE/RXNE.  Returning on TXE while STOPF is set can
     * continuously re-enter this ISR and starve USART1 and the main loop. */
    if ((sr1 & I2C_SR1_STOPF) != 0U)
    {
        I2C1->CR1 = (uint16_t)(I2C1->CR1 | I2C_CR1_PE);
        s_transmitter = 0U;
        I2C_Slave_OnStop();
        return;
    }

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

}

void I2C1_ER_IRQHandler(void)
{
    uint16_t sr1;
    uint16_t fatalErrors;

    s_bus_stall_ticks = 0U; /* any ER activity is forward progress */

    sr1 = I2C1->SR1;
    fatalErrors = (uint16_t)(sr1 & (I2C_SR1_OVR |
                                    I2C_SR1_BERR |
                                    I2C_SR1_ARLO |
                                    I2C_SR1_PECERR |
                                    I2C_SR1_TIMEOUT |
                                    I2C_SR1_SMBALERT));

    if ((sr1 & I2C_SR1_AF) != 0U)
    {
        /* NACK from master: normal end of a slave-transmitter session. */
        I2C1->SR1 &= (uint16_t)(~I2C_SR1_AF);
        s_transmitter = 0U;
        I2C_Slave_OnStop();
    }

    if ((sr1 & I2C_SR1_OVR) != 0U)
    {
        volatile uint16_t dummyStatus;
        volatile uint8_t  dummyData;

        dummyStatus = I2C1->SR1;
        dummyData   = (uint8_t)I2C1->DR;
        (void)dummyData;
        (void)dummyStatus;

        /* Clear OVR explicitly. If left set, the ER IRQ can spin forever
         * and starve the UART/main loop. */
        I2C1->SR1 &= (uint16_t)(~I2C_SR1_OVR);
        s_transmitter = 0U;
    }

    if ((sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO)) != 0U)
    {
        /* Clear both flags in one write.  ARLO has no other handler in this
         * ISR: leaving it set causes the ER ISR to re-fire immediately in a
         * tight loop (interrupt storm) that starves the main loop. */
        I2C1->SR1 &= (uint16_t)(~(I2C_SR1_BERR | I2C_SR1_ARLO));
        s_transmitter = 0U;
    }

    /* Clear every remaining enabled error source.  Any one left asserted can
     * turn the error ISR into a permanent high-priority busy loop. */
    if ((sr1 & (I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_SMBALERT)) != 0U)
    {
        I2C1->SR1 &= (uint16_t)(~(I2C_SR1_PECERR |
                                  I2C_SR1_TIMEOUT |
                                  I2C_SR1_SMBALERT));
        s_transmitter = 0U;
    }

    if (fatalErrors != 0U)
    {
        /* Stop further I2C IRQs until the main loop performs a full peripheral
         * reset. USART and the watchdog remain serviceable meanwhile. */
        I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);
        s_recovery_pending = 1U;
        I2C_Slave_OnError();
    }
}
