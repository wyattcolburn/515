/*
 * utils.h
 *
 *  Created on: May 31, 2025
 *      Author: pherder
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include "LPUART.h"
#include <stddef.h>
#include <string.h>


// Size of the array:
#define MATRIX_SIZE 25

void populate_matrix(uint8_t A[MATRIX_SIZE][MATRIX_SIZE], uint8_t B[MATRIX_SIZE][MATRIX_SIZE]);
void matrix_multiply(uint8_t A[][MATRIX_SIZE], uint8_t B[][MATRIX_SIZE], uint16_t C[][MATRIX_SIZE], int start_row, int end_row);
void print_outcome_matrix(uint16_t A[][MATRIX_SIZE], int rows);
void print_matrix(uint8_t A[][MATRIX_SIZE], int rows);
void string_to_array(char* string, uint16_t len, uint16_t* output_array);
void string_to_array_8bit(char* string, uint16_t len, uint8_t* output_array);
void init_user_button(void);
uint32_t get_user_button_state(void);
uint8_t is_pressed(void);

#endif /* INC_UTILS_H_ */
