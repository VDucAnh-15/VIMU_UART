#include "vimu_timer.h"
#include "stm32f10x_rcc.h"

static volatile uint32_t vimu_timer_tick = 0U;
static const uint8_t g_vimuApbPrescalerShift[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};

__weak void vimu_timer_tick_callback(void)
{
}

static uint32_t vimu_timer_get_tim2_clock_hz(void)
{
    uint32_t apb1Shift;
    uint32_t pclk1Hz;

    SystemCoreClockUpdate();

    apb1Shift = g_vimuApbPrescalerShift[(RCC->CFGR & RCC_CFGR_PPRE1) >> 8U];
    pclk1Hz = SystemCoreClock >> apb1Shift;

    if (apb1Shift == 0U)
    {
        return pclk1Hz;
    }

    return (pclk1Hz * 2U);
}

/**
 * @brief Initialize TIM2 with custom Period and Prescaler
 * @param psc  Prescaler value
 * @param pd   Period value
 */
void vimu_timer_init_raw(uint16_t psc, uint16_t pd)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_DeInit(TIM2);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = pd;          // Custom period
    TIM_TimeBaseStructure.TIM_Prescaler = psc;      // Custom prescaler
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    /* Kept below I2C1 (preempt 0-1, see vimu_i2c_slave.c) so a fill tick
     * never delays an in-progress I2C transaction and stretches SCL. */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

void vimu_timer_init_hz(uint32_t tickHz)
{
    uint32_t timerClockHz;
    uint32_t countsPerTick;
    uint32_t prescaler;
    uint32_t period;

    if (tickHz == 0U)
    {
        return;
    }

    timerClockHz = vimu_timer_get_tim2_clock_hz();
    countsPerTick = (timerClockHz + (tickHz / 2U)) / tickHz;
    if (countsPerTick == 0U)
    {
        countsPerTick = 1U;
    }

    prescaler = (countsPerTick - 1U) / 65536U;
    if (prescaler > 0xFFFFU)
    {
        prescaler = 0xFFFFU;
    }

    period = countsPerTick / (prescaler + 1U);
    if (period == 0U)
    {
        period = 1U;
    }
    if (period > 65536U)
    {
        period = 65536U;
    }

    vimu_timer_init_raw((uint16_t)prescaler, (uint16_t)(period - 1U));
}

uint32_t vimu_timer_get_tick(void)
{
    return vimu_timer_tick;
}

void delay_ms(uint32_t ms)
{
    uint32_t startTick;

    startTick = vimu_timer_get_tick();
    while ((vimu_timer_get_tick() - startTick) < ms)
    {
    }
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        vimu_timer_tick++;
        vimu_timer_tick_callback();
    }
}
