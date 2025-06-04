/*
 * uart.h
 *
 *  Created on: May 13, 2025
 *      Author: Wyatt
 */

#ifndef INC_UART_H_
#define INC_UART_H_

typedef struct __attribute__((packed)){
  uint8_t header; //size of the packet, type of data being sent?
  uint8_t data[MAX_DATA_SIZE];

} UART_Packet;


#define BAUDRATE 115200 // baudrate 115200 for 4Mhz
#define USARTDIV (4000000 / BAUDRATE);
//#define USARTDIV 8889
void UART3_init(void);
void USART3_TransmitString(const char* str);
void UART_Send_Packet(uint8_t * data, uint16_t data_size);
void UART3_mcu1_solo_matrix(void);
void UART3_mcu1_test(void);
void UART3_mcu2_test(void);
void UART3_mcu1_matrix(void);
void UART3_mcu2_matrix(void);



#endif /* INC_UART_H_ */
