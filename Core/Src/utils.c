/*
 * utils.c
 *
 *  Created on: May 31, 2025
 *      Author: pherder
 */

#include "utils.h"


// -- matrix
void matrix_multiply(uint8_t A[][MATRIX_SIZE], uint8_t B[][MATRIX_SIZE], uint16_t C[][MATRIX_SIZE], int start_row, int end_row) {
    for (int i = start_row; i < end_row; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            uint16_t sum = 0;
            for (int k = 0; k < MATRIX_SIZE; k++) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
}

void populate_matrix(uint8_t A[MATRIX_SIZE][MATRIX_SIZE], uint8_t B[MATRIX_SIZE][MATRIX_SIZE]){
	for (int i = 0; i < MATRIX_SIZE; i++){
		for(int j = 0; j < MATRIX_SIZE; j++){
			// basic population
			A[i][j] = (uint8_t)((i+j) % 256);
			B[i][j] = (uint8_t)((i*j) % 256);
		}
	}
}

void print_matrix(uint8_t A[][MATRIX_SIZE], int rows) {
    char buf[128];
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            sprintf(buf, "%d|", A[i][j]);
            LPUART_Print(buf);
        }
        LPUART_Print("\r\n");
    }
}

void print_outcome_matrix(uint16_t A[][MATRIX_SIZE], int rows) {
    char buf[128];
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            sprintf(buf, "%u|", A[i][j]);
            LPUART_Print(buf);
        }
        LPUART_Print("\r\n");
    }
}



// -- string conversion
void string_to_array(char* string, uint16_t len, uint16_t* output_array) {

  for (uint16_t counter = 0; counter < len; counter++ ) {
    output_array[counter] = (uint16_t)string[counter];
  }
}

void string_to_array_8bit(char* string, uint16_t len, uint8_t* output_array) {

  for (uint16_t counter = 0; counter < len; counter++ ) {
    output_array[counter] = (uint8_t)string[counter];
  }
}


// -- user button
void init_user_button(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD13);
}

uint32_t get_user_button_state(void){
    return (GPIOC->IDR & GPIO_IDR_ID13);
}


uint8_t is_pressed(void){
	// checks if button is pressed with 50ms debounce
	if(get_user_button_state() == 0){
		// debounce
		while(get_user_button_state() == 0){
		  HAL_Delay(50);
		}
	return 1;
	}
	else{
		return 0;
	}
}
