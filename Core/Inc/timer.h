/*
 * timer.h
 *
 *  Created on: May 30, 2025
 *      Author: abull
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>

void init_timer(void);
void restart_timer(void);
void start_timer(void);
uint32_t stop_timer(void);

#endif /* INC_TIMER_H_ */
