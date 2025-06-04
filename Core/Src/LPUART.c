/*
//  ******************************************************************************
//  * @file           : LPUART.c
//  * @brief          : This is the source file to initialize the LPUART on the
 *                      STM32, be able to print to the terminal, and echo from
 *                      the terminal.
 *
//  authors: Wyatt Colburn -wdcolbur@calpoly.edu
 * 			 Lin Knudsen
*/
#include "LPUART.h"
#include <stdio.h>
#include <string.h>

#define LPUART_PORT GPIOG
extern uint8_t x_cord;
extern uint8_t y_cord;

extern char UART_MESSAGE[MAX_MESSAGE_SIZE];
uint8_t bufferCount;
volatile bool messageReady;

void LPUART_Init(void) {
	PWR->CR2 |= (PWR_CR2_IOSV);              // power avail on PG[15:2] (LPUART1)
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOGEN);   // enable GPIOG clock
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN; // enable LPUART clock bridge

	/* USER: configure GPIOG registers MODER/PUPDR/OTYPER/OSPEEDR then
	   select AF mode and specify which function with AFR[0] and AFR[1] */
	LPUART_PORT->MODER &= ~(GPIO_MODER_MODE7 | GPIO_MODER_MODE8);
			// Set MODE4/5/7 to 10 (alternate function)
	LPUART_PORT->MODER |= (GPIO_MODER_MODE7_1 | GPIO_MODER_MODE8_1);
		// Push-pull (0)
	LPUART_PORT->OTYPER &= ~(GPIO_OTYPER_OT7 | GPIO_OTYPER_OT8);
		// Low speed (0)
	LPUART_PORT->OSPEEDR |= (GPIO_OSPEEDR_OSPEED7 | GPIO_OSPEEDR_OSPEED8);
		// No pull up / no pull down (00)
	LPUART_PORT->PUPDR &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8);
	LPUART_PORT->AFR[0] &= ~((0x000F << GPIO_AFRL_AFSEL7_Pos));
	LPUART_PORT->AFR[1] &= ~((0x000F  << GPIO_AFRH_AFSEL8_Pos));
	LPUART_PORT->AFR[0] |=  (8 << GPIO_AFRL_AFSEL7_Pos);
	LPUART_PORT->AFR[1] |=  (8 << GPIO_AFRH_AFSEL8_Pos);
	//Needs to happen before enable LPUART

	//RCC->CCIPR |= (RCC_CCIPR_LPUART1SEL_1);    //Sets CLK used to System CLK--4MHZ
	 LPUART1->BRR = (0xFFFFF); //20 bits
	 LPUART1->BRR = (8889);                    //Sets baud rate to 115.2K

	LPUART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0); // 8-bit data
	LPUART1->CR1 |= USART_CR1_UE;                   // enable LPUART1
	LPUART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);  // enable xmit & recv
	LPUART1->CR1 |= USART_CR1_RXNEIE;        // enable LPUART1 recv interrupt
	LPUART1->ISR &= ~(USART_ISR_RXNE);       // clear Recv-Not-Empty flag
	/* USER: set baud rate register (LPUART1->BRR) */

	NVIC->ISER[2] = (1 << (LPUART1_IRQn & 0x1F));   // enable LPUART1 ISR
	__enable_irq();                          // enable global interrupts)

	LPUART_ESC_Print(CLEAR_SCREEN);
	LPUART_ESC_Print(CURSOR_HOME);
	LPUART_ESC_Print(SET_COLOR(WHITE));
}

void LPUART_Print( const char* message ) {
   uint16_t iStrIdx = 0;
   while ( message[iStrIdx] != 0 ) {
      while(!(LPUART1->ISR & USART_ISR_TXE)) // wait for empty xmit buffer
         ;
      LPUART1->TDR = message[iStrIdx];       // send this character
	iStrIdx++;                             // advance index to next char
   }
}

void LPUART_print_uint8_matrix(uint8_t *buffer, int rows, int cols) {
    char buf[32];
    int index = 0;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            sprintf(buf, "%d|", buffer[index]);
            LPUART_Print(buf);
            index++;
        }
        LPUART_Print("\r\n");
    }
    LPUART_Print("\r\n");
}


void LPUART_ESC_Print( const char* message ) {
	const char * escChar = "\x1B"; // Escape character
	LPUART_Print(escChar);
	LPUART_Print(message);
}

void LPUART1_IRQHandler( void  ) {
   uint8_t charRecv;
   if (LPUART1->ISR & USART_ISR_RXNE) {
      charRecv = LPUART1->RDR;
      switch ( charRecv ) {

	   case '\r':
	       if (bufferCount > 0) {
	   	     UART_MESSAGE[bufferCount] ='\0';
	   	     messageReady = true;
	   	     bufferCount = 0;
	       }
	       break;
	   default:
	     if (bufferCount < MAX_MESSAGE_SIZE) {
	     		  UART_MESSAGE[bufferCount] = charRecv;
	     		  bufferCount++;
	     	      }
	      while( !(LPUART1->ISR & USART_ISR_TXE) )
               ;    // wait for empty TX buffer
	      LPUART1->TDR = charRecv;  // echo char to terminal

	      break;
	}  // end switch
   }
}

void print_uint16(uint16_t value) {
    char buffer[10]; // Buffer large enough to hold any uint16_t value as string

    // Format the number as a string
    sprintf(buffer, "%u", value);

    // Print the string
    LPUART_Print(buffer);
}
