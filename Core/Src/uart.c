
#include "LPUART.h"
void UART_init(void) {
    // Using UART1: PA9 (TX), PA10 (RX)

    // Enable GPIOA and USART1 clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;   // Enable GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1

    // Configure PA9 (TX) and PA10 (RX) as alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER |= (GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1);

    // Configure as push-pull
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);

    // Set high speed
    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10);

    // Configure pull-up for RX
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD10);
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD10_0;  // Pull-up on PA10 (RX)

    // Set alternate function 7 for PA9 and PA10
    GPIOA->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL9_Pos);
    GPIOA->AFR[1] &= ~(0xF << GPIO_AFRH_AFSEL10_Pos);
    GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL9_Pos);   // AF7 for PA9
    GPIOA->AFR[1] |= (7 << GPIO_AFRH_AFSEL10_Pos);  // AF7 for PA10

    // Configure USART1
    USART1->BRR = 8889;  // Set baud rate to 115.2K (assuming appropriate clock)

    // Configure 8 data bits, 1 stop bit
    USART1->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0);  // 8 bits
    USART1->CR2 &= ~USART_CR2_STOP;                 // 1 stop bit

    // Enable RXNE interrupt
    USART1->CR1 |= USART_CR1_RXNEIE;

    // Clear flags
    USART1->ISR &= ~USART_ISR_RXNE;

    // Enable transmitter and receiver
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    // Enable USART1
    USART1->CR1 |= USART_CR1_UE;

    // Enable USART1 interrupt in NVIC
    NVIC->ISER[1] = (1 << (USART1_IRQn & 0x1F));  // USART1_IRQn is 37

    // Enable global interrupts
    __enable_irq();
}

void USART1_IRQHandler(void){

  uint8_t charRecv;
  if (USART1->ISR & USART_ISR_RXNE) {
      charRecv = USART1->RDR;
      switch (charRecv) {
	case '\r':
	  LPUART_Print("Enter");
	  break;

	default:
	  while(!(USART1->ISR & USART_ISR_TXE));
	  LPUART_Print(charRecv);
	  break;
      }
  }
}
