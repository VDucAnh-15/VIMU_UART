#include "stm32f10x.h"
#include "vimu_app.h"
#include "vimu_i2c_slave.h"
#include "vimu_timer.h"
#include "vimu_uart.h"

int main(void)
{
    vimu_app_init();
    vimu_uart_init(115200U, 0U);
    I2C_Slave_Init(VIMU_I2C_DEFAULT_ADDRESS);
    vimu_timer_init_hz(VIMU_TIMER_FILL_RATE_HZ);

    while (1)
    {
        vimu_app_service_runtime();
        vimu_app_process_uart_frame();
        vimu_app_service_runtime();
        vimu_app_service_events();
    }
}
