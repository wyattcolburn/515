#include "main.h"
#include "LPUART.h"
#include <stdio.h>
#include <string.h>
#include "utils.h"
#include "uart.h"
#include "timer.h"

volatile bool UART_dataReceived = false;
volatile bool UART_header_packet = false;
volatile bool UART_receive_done = false;
uint8_t UART_packet_data[MAX_DATA_SIZE];
uint16_t UART_packet_len;
uint16_t UART_packet_counter = 0;
size_t UART_total_counter = 0;

#define MAX_BUFFER_SIZE 40 * 1024 // need this much memory to be able to have 200x200 matrix
uint8_t UART_total_buffer[MAX_BUFFER_SIZE];
// i believe this is for input?
char UART_MESSAGE[MAX_MESSAGE_SIZE];


void reset_buffer(void){
	memset(UART_total_buffer, 0, MAX_BUFFER_SIZE);
	UART_total_counter = 0;
}

void send_ack(void){
	// wait for uart to open up
	while(!(USART3->ISR & USART_ISR_TXE));
	char * ack = "ack";
	uint8_t ack_arr[strlen(ack)];
	string_to_array_8bit(ack, strlen(ack), ack_arr);
	UART_Send_Packet(ack_arr, strlen(ack));
}

void wait_for_ack(void){
	while(1){
		if(UART_dataReceived == true){
			UART_dataReceived = false;
			// ensure that the message recv is an ack

			if(strcmp((char *)UART_total_buffer, "ack") == 0){
				// before we return, lets clear our buffer
//				LPUART_Print("RECV ACK!\r\n");
				reset_buffer();
				return;
			}
			else {
				LPUART_Print("Expected 'ack' but got something else.\r\nLooping forever....\r\n");
				while(1); // death
			}
		}
	}
}

void UART3_mcu1_solo_matrix(void){
	start_timer();
	uint8_t A[MATRIX_SIZE][MATRIX_SIZE];
	uint8_t B[MATRIX_SIZE][MATRIX_SIZE];
	uint16_t C[MATRIX_SIZE][MATRIX_SIZE];
	populate_matrix(A, B);
	matrix_multiply(A, B, C, 0, MATRIX_SIZE);
	uint32_t t = stop_timer();
	char str[100];
	sprintf(str, "Total Time: %lu\r\n", t);
	LPUART_Print(str);
}

void UART3_mcu1_matrix(void){
	/*
	 * Performs matrix mult
	 */
	reset_buffer();
	uint8_t A[MATRIX_SIZE][MATRIX_SIZE];
	uint8_t B[MATRIX_SIZE][MATRIX_SIZE];
	uint16_t C[MATRIX_SIZE][MATRIX_SIZE];

	populate_matrix(A, B);

//	// DEBUG PRINT MATRIX:
//	print_matrix(A);
//	LPUART_Print("\r\n\r\n");
//	print_matrix(B);

	// split matrix in half (splitting rows)
	uint8_t A_top[MATRIX_SIZE/2][MATRIX_SIZE];
	uint8_t A_bot[MATRIX_SIZE/2][MATRIX_SIZE];

	// Fill in the halfs
	memcpy(A_top, A, sizeof(A_top));
	memcpy(A_bot, &A[MATRIX_SIZE/2][0], sizeof(A_bot));

	//	// DEBUG PRINT MATRIX:
//	print_matrix(A_bot, MATRIX_SIZE / 2);
//	LPUART_Print("\r\n\r\n");
//	print_matrix(A, MATRIX_SIZE);

	// send over bottom half to MCU2
	UART_Send_Packet((uint8_t *)A_bot, sizeof(A_bot));

	// wait for ack
	wait_for_ack();

	// send over B.
	UART_Send_Packet((uint8_t *)B, sizeof(B));

	// do calculations.
	matrix_multiply(A_top, B, C, 0, MATRIX_SIZE / 2);

	// wait for C.
	while(1){
		if(UART_dataReceived == true){
			UART_dataReceived = false;
			// copy the bottom half into C
			memcpy(&C[MATRIX_SIZE/2][0], UART_total_buffer, (MATRIX_SIZE/2) * MATRIX_SIZE * sizeof(uint16_t));
			reset_buffer();
			send_ack();
			break;
		}
	}

	// ok done
//	print_outcome_matrix(C, MATRIX_SIZE);
	LPUART_Print("Done.\r\n");
}

void UART3_mcu2_matrix(void){
	reset_buffer();
	uint8_t a_bot[MATRIX_SIZE / 2][MATRIX_SIZE];
	uint8_t b_full[MATRIX_SIZE][MATRIX_SIZE];

	// wait for A (bottom half)
	while(1){
		if(UART_dataReceived == true){
			// A received...
			UART_dataReceived = false;
			memcpy(a_bot, UART_total_buffer, sizeof(a_bot));
			break;
		}
	}
	// send ack to tell MCU1 to send over B.
	// TODO start timer
	send_ack();
	// TODO end timer
	// wait for B (full matrix)
	// first, reset our buffer
	reset_buffer();
	// now wait for B.
	while(1){
		if(UART_dataReceived == true){
			UART_dataReceived = false;
			// Copy over
			memcpy(b_full, UART_total_buffer, sizeof(b_full));
			break;
		}
	}
	// OK we should have both now, we can do the comp
//	LPUART_print_uint8_matrix(a_bot, MATRIX_SIZE / 2, MATRIX_SIZE);
//	LPUART_print_uint8_matrix(b_full, MATRIX_SIZE, MATRIX_SIZE);

	uint16_t c_bot[MATRIX_SIZE / 2][MATRIX_SIZE];
	matrix_multiply(a_bot, b_full, c_bot, 0, MATRIX_SIZE / 2);

	// ok we finished C, lets send it back to MCU1
	UART_Send_Packet((uint8_t *)c_bot, sizeof(c_bot));
	wait_for_ack();
	// I think we are good to exit now.
	LPUART_Print("Done.\r\n");
	return;
}


void UART3_mcu1_test(void){
	/*
	 * Tests the bidirectional characteristics of UART with
	 * string tx and rx
	 */
	// Create message
	memset(UART_total_buffer, 0, MAX_BUFFER_SIZE);
	UART_total_counter = 0;
	char* uart_message = "Maman died today. Or yesterday maybe, I don't know. I got a telegram from the home: Mother deceased. Funeral Tomorrow. Faithfully yours. That doesn't mean anything. Maybe it was yesterday. The old people's home is at Marengo, about eighty kilometers from Algiers, I'll take the two o'clock bus and get there in the afternoon. That way I can be there for the vigil and come back tomorrow night. i miss the the fortnite bus";
	size_t len = strlen(uart_message);
    uint8_t output_array[len];
	string_to_array_8bit(uart_message, len, output_array);
	// Send packet
	UART_Send_Packet(output_array, len);
	// wait for ack
	while(1){
		LPUART_Print("Waiting for ack...   ");
		if(UART_dataReceived == true){
			UART_dataReceived = false;
//			LPUART_Print("recv message: ");
			LPUART_Print((char *) UART_total_buffer);
			break;
		}
		HAL_Delay(500);
	}
}


void UART3_mcu2_test(void){
	memset(UART_total_buffer, 0, MAX_BUFFER_SIZE);
	UART_total_counter = 0;
	while(1){
//		LPUART_Print("Waiting for data...   ");
		if(UART_dataReceived == true){
			UART_dataReceived = false; // clear bool for next message
//			LPUART_Print("recv message: ");
			// perform manipulation
			LPUART_Print((char *) UART_total_buffer);
			LPUART_Print("\r\n");

			// send ack
			char * ack = "recv";
			uint8_t ack_arr[strlen(ack)];
			string_to_array_8bit(ack, strlen(ack), ack_arr);
			UART_Send_Packet(ack_arr, strlen(ack));
			break;
		}
		HAL_Delay(500);
	}
}


void USART3_IRQHandler(void) {
	if (USART3->ISR & USART_ISR_FE) {
//	    LPUART_Print("FRAMING ERROR!\r\n");
	    USART3->ICR |= USART_ICR_FECF;   // clear framing error
	}

    if (USART3->ISR & USART_ISR_RXNE) {
        volatile uint8_t charRecv = USART3->RDR;
//        char str[30];
        if (UART_header_packet == false) {
            // new packet starting
            UART_packet_len = charRecv;
            UART_header_packet = true;
            UART_packet_counter = 0;
//            sprintf(str, "HEADER: %d\r\n", UART_packet_len);

            // Special case: end packet
            if (UART_packet_len == 0) {
            	UART_total_counter = 0;
                UART_dataReceived = true;
                UART_header_packet = false;
//                sprintf(str, "MSG: END\r\n");
//                LPUART_Print(str);
                return;
            }
        }
        else {
            // receiving DATA
            UART_total_buffer[UART_total_counter] = charRecv;
//            sprintf(str, "  DATA: %c|%u (%d)\r\n", charRecv, charRecv, UART_packet_counter);
            UART_packet_counter++;
            UART_total_counter ++;

            // If end of this packet reached
            if (UART_packet_counter == UART_packet_len) {
                UART_header_packet = false;
                UART_packet_counter = 0;
            }
        }
//        LPUART_Print(str);

    }
}



void UART3_init(void) {
    // Using USART3: PB10 (TX), PB11 (RX)
    // Enable GPIOB and USART3 clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;   // Enable GPIOB
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN; // Enable USART3 (on APB1)

    // Configure PB10 (TX) and PB11 (RX) as alternate function
    GPIOB->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11);
    GPIOB->MODER |= (GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1);

    // Configure as push-pull
    GPIOB->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11);

    // Set high speed
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11);

    // Configure pull-up for RX
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD11);
    GPIOB->PUPDR |= GPIO_PUPDR_PUPD11_0;  // Pull-up on PB11 (RX)

    // Set alternate function 7 for PB10 and PB11
    GPIOB->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL10_Pos);
    GPIOB->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL11_Pos);
    GPIOB->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);   // AF7 for PB10
    GPIOB->AFR[1] |= (7 << GPIO_AFRH_AFSEL11_Pos);   // AF7 for PB11

    // Configure USART3
    USART3->BRR = 2963;

    // Configure 8 data bits, 1 stop bit
    USART3->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);  // 8 bits
    USART3->CR2 &= ~USART_CR2_STOP;                 // 1 stop bit

    // Enable RXNE interrupt
    USART3->CR1 |= USART_CR1_RXNEIE;

    // Clear flags
    USART3->ISR &= ~USART_ISR_RXNE;

    // Enable transmitter and receiver
    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable USART3
    USART3->CR1 |= USART_CR1_UE;

    // Enable USART3 interrupt in NVIC
    NVIC->ISER[1] = (1 << (USART3_IRQn & 0x1F));  // USART3_IRQn is 39

    // Enable global interrupts
    __enable_irq();
}

void USART3_TransmitString(const char* str) {
    while (*str) {
        while(!(USART3->ISR & USART_ISR_TXE));
        USART3->TDR = *str++;
    }
    // Wait for last byte to complete
    while(!(USART3->ISR & USART_ISR_TC));
}

void UART_Send_Packet(uint8_t *data, uint16_t data_size) {

  bool sending;
  sending = true;
  size_t amount_to_send = data_size;

  while (sending) {

      if (amount_to_send == 0) {
	  break;
      }

      size_t bytes_sending = (MAX_DATA_SIZE < amount_to_send) ? MAX_DATA_SIZE : amount_to_send;

      UART_Packet current_packet;
      current_packet.header = (uint8_t)bytes_sending;

      memcpy(current_packet.data, &data[data_size - amount_to_send], current_packet.header);
      amount_to_send -= bytes_sending;

      while(!(USART3->ISR & USART_ISR_TXE)){};
	  USART3->TDR = current_packet.header;
//             LPUART_Print("Header is : ");
//                   print_uint16(current_packet.header);
//                   LPUART_Print("   \n");


      for (uint8_t transmission_counter = 0; transmission_counter < current_packet.header; transmission_counter++) {
	  while(!(USART3->ISR & USART_ISR_TXE)); //making sure previous transmission has completed
	  USART3->TDR = current_packet.data[transmission_counter];
      }

//      HAL_Delay(100);
  }
  while(!(USART3->ISR & USART_ISR_TXE)); //making sure previous transmission has completed
  USART3->TDR = 0x00;
  //send 0000
}
