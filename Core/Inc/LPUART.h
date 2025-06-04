#include <stdint.h>
#include "main.h"
#ifndef INC_LPUART_H_
#define INC_LPUART_H_

// Macros to make SET_COLOR work

#define TO_TEXT(str) #str
#define _SET_COLOR_HELPER(color) "[3" #color "m"
#define SET_COLOR(color) _SET_COLOR_HELPER(color)

// Color macros
#define RED 1
#define GREEN 2
#define BLUE 4
#define WHITE 7

// Escape sequences for special commands
#define CLEAR_SCREEN "[2J"
#define CHAR_ATTRIB_OFF "[0m"
#define CURSOR_HOME "[H"
#define CURSOR_DOWN(n) "[" #n "B"
#define CURSOR_UP(n) "[" #n "A"
#define CURSOR_LEFT(n) "[" #n "D"
#define CURSOR_RIGHT(n) "[" #n "C"
#define CURSOR_SET_POS(row,col) "[" #row ";" #col "H"

#define NUM_ROW 40
#define NUM_COL 80


// Declare the buffer and related variables as external



void LPUART_Init(void);
void LPUART_Print(const char* message);
void LPUART_print_uint8_matrix(uint8_t *buffer, int rows, int cols);
void LPUART_ESC_Print(const char* message);
void delay_us(const uint32_t);
void print_uint16(uint16_t value);

#endif /* INC_LPUART_H_ */
