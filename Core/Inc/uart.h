/*
 * uart.h
 *
 *  Created on: May 13, 2025
 *      Author: Wyatt
 */

#ifndef INC_UART_H_
#define INC_UART_H_

typedef struct {
  uint8_t header; //size of the packet, type of data being sent?
  uint8_t data[MAX_DATA_SIZE];

} UART_Packet;

void UART3_init(void);
void USART3_TransmitString(const char* str);
void UART_Send_Packet(uint8_t * data, uint16_t data_size);

#endif /* INC_UART_H_ */
