#ifndef VIMU_APP_H
#define VIMU_APP_H

#include "vimu_fifo.h"
#include "vimu_message_uart.h"

#define VIMU_TOTAL_SAMPLE_CAPACITY    250U
#define VIMU_LOW_WATERMARK_SAMPLES    50U
#define VIMU_STATUS_PAYLOAD_SIZE      3U
#define VIMU_RING_STORAGE_SIZE        ((VIMU_TOTAL_SAMPLE_CAPACITY * VIMU_UART_SAMPLE_SIZE) + 1U)
#define VIMU_TIMER_PRESCALER_50HZ     1439U
#define VIMU_TIMER_PERIOD_50HZ        999U

void vimu_app_init(void);
void vimu_app_process_uart_frame(void);
void vimu_app_service_runtime(void);
void vimu_app_service_events(void);

#endif /* VIMU_APP_H */
