#include "stm32f10x.h"
#include "vimu_app.h"
#include "vimu_timer.h"
#include "vimu_uart.h"

int main(void)
{
    vimu_app_init();
    vimu_uart_init(115200U, 0U);
    vimu_timer_init(VIMU_TIMER_PRESCALER_50HZ, VIMU_TIMER_PERIOD_50HZ);

    while (1)
    {
        vimu_app_service_runtime();
        vimu_app_process_uart_frame();
        vimu_app_service_runtime();
        vimu_app_service_events();
    }
}
