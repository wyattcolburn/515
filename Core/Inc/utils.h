/*
 * utils.h
 *
 *  Created on: May 31, 2025
 *      Author: pherder
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>

void string_to_array(char* string, uint16_t len, uint16_t* output_array);
void string_to_array_8bit(char* string, uint16_t len, uint8_t* output_array);


#endif /* INC_UTILS_H_ */
