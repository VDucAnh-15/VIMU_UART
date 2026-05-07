#ifndef _VIMU_TIMER_H_
#define _VIMU_TIMER_H_

#include "stm32f10x.h"                  // Device header
#include "stm32f10x_tim.h"              // Timer header

/**
 * @brief Initialize TIM4 with custom Period and Prescaler
 * @param psc  Prescaler value (0-65535)
 * @param pd   Period value (0-65535)
 * 
 * Frequency = SystemCoreClock / ((psc + 1) * (pd + 1))
 * Example:
 *   50Hz:  vimu_timer_init(1439, 999)  => 72MHz / (1440 * 1000) = 50Hz
 *   100Hz: vimu_timer_init(719, 999)   => 72MHz / (720 * 1000) = 100Hz
 */
void vimu_timer_init(uint16_t psc, uint16_t pd);
void delay_ms(uint32_t ms);



#endif /* _VIMU_TIMER_H_ */
