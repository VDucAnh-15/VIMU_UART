#ifndef _VIMU_TIMER_H_
#define _VIMU_TIMER_H_

#include "stm32f10x.h"                  // Device header
#include "stm32f10x_tim.h"              // Timer header

/**
 * @brief Initialize TIM2 with custom Period and Prescaler
 * @param psc  Prescaler value (0-65535)
 * @param pd   Period value (0-65535)
 */
void vimu_timer_init_raw(uint16_t psc, uint16_t pd);

/**
 * @brief Initialize TIM2 to generate an update event at the requested frequency.
 * @param tickHz  Target interrupt frequency in Hz
 * 
 * The timer clock is derived from the live RCC configuration so the result
 * stays correct even if the MCU falls back to HSI instead of running at the
 * nominal PLL frequency.
 */
void vimu_timer_init_hz(uint32_t tickHz);
uint32_t vimu_timer_get_tick(void);
void vimu_timer_tick_callback(void);
void delay_ms(uint32_t ms);
#endif /* _VIMU_TIMER_H_ */
