#include "main.h"
#include "LPUART.h"
#include <stdio.h>
#include <string.h>
#include "utils.h"
#include "uart.h"

volatile bool UART_dataReceived = false;
volatile bool UART_header_packet = false;
volatile bool UART_receive_done = false;
uint8_t UART_packet_data[MAX_DATA_SIZE];
uint16_t UART_packet_len;
uint16_t UART_packet_counter = 0;
size_t UART_total_counter = 0;

#define MAX_BUFFER_SIZE 4096
uint8_t UART_total_buffer[MAX_BUFFER_SIZE];
// i believe this is for input?
char UART_MESSAGE[MAX_MESSAGE_SIZE];


void UART3_mcu1_test(void){
	/*
	 * Tests the bidirectional characteristics of UART with
	 * string tx and rx
	 */
	// Create message
	char* uart_message = "Maman died today. Or yesterday maybe, I don't know. I got a telegram from the home: Mother deceased. Funeral Tomorrow. Faithfully yours. That doesn't mean anything. Maybe it was yesterday. The old people's home is at Marengo, about eighty kilometers from Algiers, I'll take the two o'clock bus and get there in the afternoon. That way I can be there for the vigil and come back tomorrow night. I asked my boss for two days off and there was no way";
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

    if (USART3->ISR & USART_ISR_RXNE) {
        volatile uint8_t charRecv = USART3->RDR;
        char str[30];
        if (UART_header_packet == false) {
            // new packet starting
            UART_packet_len = charRecv;
            UART_header_packet = true;
            UART_packet_counter = 0;  // reset counter immediately!

            sprintf(str, "HEADER: %d\r\n", UART_packet_len);

            // Special case: end packet
            if (UART_packet_len == 0) {
            	UART_total_counter = 0;
                UART_dataReceived = true;
                UART_header_packet = false;
                sprintf(str, "MSG: END\r\n");
                LPUART_Print(str);
                return;
            }
        }
        else {
            // receiving DATA
            UART_total_buffer[UART_total_counter] = charRecv;
            sprintf(str, "  DATA: %c (%d)\r\n", charRecv, UART_packet_counter);
            UART_packet_counter++;
            UART_total_counter ++;

            // If end of this packet reached
            if (UART_packet_counter == UART_packet_len) {
                UART_header_packet = false;
                UART_packet_counter = 0;
            }
        }
        LPUART_Print(str);

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
    USART3->BRR = 8889;  // Set baud rate to 115.2K (adjust based on your clock)

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
