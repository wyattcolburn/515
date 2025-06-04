/*
 * timer.c
 *
 *  Created on: May 30, 2025
 *      Author: abull
 */
#include "timer.h"
#include "main.h"
#include "LPUART.h"
#include <stdio.h>

void init_timer(void) {
   RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
   TIM2->PSC = 3; // 1us per tick (apparently)
   TIM2->ARR = 0xFFFFFFFF;
   TIM2->CNT = 0;
   TIM2->CR1 &= ~TIM_CR1_DIR;
}

void restart_timer(void){
	// Sets count to 0 and starts timer
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN; // enable
}

void start_timer(void){
	TIM2->CR1 |= TIM_CR1_CEN; // enable
}

uint32_t stop_timer(void){
	// Stops timer and returns current timer value
	uint32_t ret = TIM2->CNT;
	TIM2->CR1 &= ~TIM_CR1_CEN; // enable
	return ret / 4;
}

void print_timer(void){
	// prints current value within timer
	uint32_t t = TIM2->CNT / 4;
	char str[50];
	sprintf(str, "TIMER (us): %lu", t);
	LPUART_Print(str);
}


