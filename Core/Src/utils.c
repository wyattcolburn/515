/*
 * utils.c
 *
 *  Created on: May 31, 2025
 *      Author: pherder
 */

#include "utils.h"
#include <stdint.h>

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


