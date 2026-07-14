#include "stm32f10x.h"
#include "vimu_app.h"
#include "vimu_i2c_slave.h"
#include "vimu_timer.h"
#include "vimu_uart.h"

static void vimu_watchdog_init(void)
{
    /* LSI is typically 40 kHz. PR=/256 and RLR=624 gives about 4 seconds. */
    IWDG->KR = 0x5555U;
    IWDG->PR = 6U;
    IWDG->RLR = 624U;
    IWDG->KR = 0xAAAAU;
    IWDG->KR = 0xCCCCU;
}

static void vimu_watchdog_kick(void)
{
    IWDG->KR = 0xAAAAU;
}

int main(void)
{
    vimu_app_init();
    vimu_uart_init(115200U, 0U);
    I2C_Slave_Init(VIMU_I2C_DEFAULT_ADDRESS);
    vimu_timer_init_hz(VIMU_TIMER_FILL_RATE_HZ);
    vimu_watchdog_init();

    while (1)
    {
        I2C_Slave_Service();
        vimu_app_service_runtime();
        vimu_app_process_uart_frame();
        vimu_app_service_runtime();
        vimu_app_service_events();
        vimu_watchdog_kick();
    }
}
