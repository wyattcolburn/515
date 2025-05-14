#include "main.h"
#include "LPUART.h"
#include "uart.h"
volatile bool header_packet;
volatile bool receive_done;
volatile uint16_t packet_data[4096];
volatile uint16_t packet_len;
volatile uint16_t packet_counter = 0;
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

void USART3_IRQHandler(void) {

    if (USART3->ISR & USART_ISR_RXNE) {
        volatile uint8_t charRecv = USART3->RDR;

        if (header_packet == false) {
            packet_len = charRecv;
            header_packet = true;
            if (packet_len == 0) {
        	dataReceived = true;
        	header_packet = false;
            }
        }

        else {
            packetData[packet_counter] = charRecv;
            packet_counter++;
            if (packet_count == packet_len) {
        	header_packet = false;
        	dataReceived = true;
        	packet_counter = 0;
            }
        }

    }
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
      while(!(USART3->ISR & USART_ISR_TXE));
             USART3->TDR = current_packet.header;
             LPUART_Print("Header is : ");
                   print_uint16(current_packet.header);
                   LPUART_Print("   \n");


      for (uint8_t transmission_counter = 0; transmission_counter < current_packet.header; transmission_counter++) {
	  while(!(USART3->ISR & USART_ISR_TXE)); //making sure previous transmission has completed
	  USART3->TDR = current_packet.data[transmission_counter];
	  LPUART_Print("Value : ");
	   print_uint16(current_packet.data[transmission_counter]);
	   LPUART_Print("   \n");
      }


  }
  while(!(USART3->ISR & USART_ISR_TXE)); //making sure previous transmission has completed
  USART3->TDR = 0x00;
  //send 0000
}
