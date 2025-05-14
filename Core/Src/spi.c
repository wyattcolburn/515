//Wyatt and Franco:
// Spi functions
#include "main.h" //??
#include "spi.h"
#include "LPUART.h"

extern bool dataReceived;


volatile bool header_packet;
volatile bool receive_done;
volatile uint16_t packet_data[4096];
volatile uint16_t packet_len;
volatile uint16_t packet_counter = 0;


//MCU 1 is the master, SPI1 is sending to
//MCU 2 slave, receives on SPI2

//Just SPI1

void SPI_Master_Init( void) {

   RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
   RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOEEN);

   GPIOE->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE13 | GPIO_MODER_MODE15);
   // set MODE4/5/7 to 10 (alternate function)
   GPIOE->MODER |= (GPIO_MODER_MODE12_1 | GPIO_MODER_MODE13_1 | GPIO_MODER_MODE15_1);
   // push-pull (0)
   GPIOE->OTYPER &= ~(GPIO_OTYPER_OT12 | GPIO_OTYPER_OT13 | GPIO_OTYPER_OT15);
   // low speed (0)
   GPIOE->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED12 | GPIO_OSPEEDR_OSPEED13 | GPIO_OSPEEDR_OSPEED15);
   // no pull up / no pull down (00)
   GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD12 | GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD15);

   GPIOE->AFR[1] |= (5 << GPIO_AFRH_AFSEL12_Pos |
					  5 << GPIO_AFRH_AFSEL13_Pos | //SCL
					  5 << GPIO_AFRH_AFSEL15_Pos); //MOSI

   SPI1->CR1 &= ~( SPI_CR1_SPE );             	// disable SPI for config
   SPI1->CR1 &= ~( SPI_CR1_RXONLY );          	// recv-only OFF
   SPI1->CR1 &= ~( SPI_CR1_LSBFIRST );        	// data bit order MSb:LSb
   SPI1->CR1 &= ~( SPI_CR1_CPOL | SPI_CR1_CPHA ); // SCLK polarity:phase = 0:0
   SPI1->CR1 |=	 (SPI_CR1_MSTR);              	// MCU is SPI Slave
   SPI1->CR1 |= SPI_CR1_BR_1;  // Divide clock by 8


   // CR2 (reset value = 0x0700 : 8b data)
   SPI1->CR2 &= ~( SPI_CR2_TXEIE | SPI_CR2_RXNEIE ); // disable FIFO intrpts
   SPI1->CR2 &= ~( SPI_CR2_FRF);              	// Moto frame format
   SPI1->CR2 |=	 SPI_CR2_NSSP;              	// auto-generate NSS pulse
   SPI1->CR2 |=	 SPI_CR2_DS;                	// 16-bit data
   SPI1->CR2 |=	 SPI_CR2_SSOE;              	// enable SS output
   // CR1
   SPI1->CR1 |=	 SPI_CR1_SPE;               	// re-enable SPI for ops
}

void SPI_Slave_Init(void) {
  // SPI2 uses PD0 for NSS, PD1 for SCL, PD4 for MOSI

  // Enable clocks
  RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;  // SPI2 is on APB1, not APB2
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;

  // Configure GPIO pins
  GPIOD->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE4);
  // Set to alternate function mode (10)
  GPIOD->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE4_1);

  // Set to push-pull output type (0)
  GPIOD->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT4);

  // Set to low speed (0)
  GPIOD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED4);

  // No pull-up/pull-down (00)
  GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD4);

  // Set alternate function - use AFRL for pins 0-7
  GPIOD->AFR[0] |= (5 << GPIO_AFRL_AFSEL0_Pos |
                    5 << GPIO_AFRL_AFSEL1_Pos |
                    5 << GPIO_AFRL_AFSEL4_Pos);

  // Configure SPI2 for slave mode
  SPI2->CR1 &= ~(SPI_CR1_SPE);             // Disable SPI for config
  SPI2->CR1 &= ~(SPI_CR1_RXONLY);          // Recv-only OFF
  SPI2->CR1 &= ~(SPI_CR1_LSBFIRST);        // Data bit order MSb:LSb
  SPI2->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); // SCLK polarity:phase = 0:0
  SPI2->CR1 &= ~(SPI_CR1_MSTR);            // MCU is SPI Slave



  // Configure CR2
  SPI2->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE); // Disable FIFO interrupts
  SPI2->CR2 &= ~(SPI_CR2_FRF);             // Motorola frame format
  SPI2->CR2 &= ~(SPI_CR2_NSSP);            // No NSS pulse in slave mode
  SPI2->CR2 |= SPI_CR2_DS;                 // 16-bit data
  //SPI2->CR2 |= (SPI_CR2_SSOE);            // Disable SS output in slave mode
  SPI2->CR2 &= ~(SPI_CR2_DS);              // Clear DS bits first
  SPI2->CR2 |= (0xF << SPI_CR2_DS_Pos);    // Set to 16-bit data (0xF is 1111 binary)

  // Disable SS output in slave mode (this is a correction)
  SPI2->CR2 &= ~(SPI_CR2_SSOE);

  // Enable SPI
  SPI2->CR1 |= SPI_CR1_SPE;                // Re-enable SPI for ops

  SPI2->CR2 |= SPI_CR2_RXNEIE; //enable receive interrupt
  NVIC_SetPriority(SPI2_IRQn,1);  	   //enabling receive interupt
  NVIC_EnableIRQ(SPI2_IRQn);
}
void SPI2_IRQHandler(void) {
  if (SPI2->SR & SPI_SR_RXNE) { //only happens when NSS is low?
      volatile uint16_t received_byte = SPI2->DR;

      // Add NSS status check
      bool nss_active = !(GPIOD->IDR & GPIO_IDR_ID0);  // Assuming NSS is on PD0, 1 for HIGh, 0 if LOW

      // If NSS is inactive (high), reset state machine
      if (!nss_active) {
          header_packet = false;
          packet_counter = 0;

          LPUART_Print("NSS gets hit");
          return;  // Skip processing this byte
      }

      // Normal protocol handling (unchanged)
      if (header_packet == false) {
          packet_len = received_byte;
          header_packet = true;
          if (packet_len == 0) {
              dataReceived = true;
              header_packet = false;
          }
      }
      else {
          packet_data[packet_counter] = received_byte;
          packet_counter++;
          if (packet_counter == packet_len) {
              header_packet = false;
              dataReceived = true;
              packet_counter = 0;
          }
      }
  }
}

void MCU_1_Main(void) {

  char* message = "hello franco";
  uint16_t len = strlen(message);
  uint16_t output_Array[len];
  string_to_array(message, len, output_Array);

  SPI_Send_Packet(output_Array, len);

  //7 transmissions: Header which is 6
  // Data: 0,1,2,3,4,5
  // Ender: 0000

}

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
void MCU_2_Main(void) {
  //receive the packet

  if (dataReceived) {
      LPUART_Print("Packet has been received : ");

      for (int i = 0; i < packet_len; i++) {
	 print_uint16(packet_data[i]);
      }
      LPUART_Print("\r\n");
      dataReceived = false;

  }
}

uint16_t SPI_Read_From_Peer(void) {
    // Wait until receive buffer contains data
    while(!(SPI2->SR & SPI_SR_RXNE));

    // Return received data
    return SPI2->DR;
}

void SPI_Send_Packet(uint16_t *data, uint16_t data_size) { //first item of the array

  bool sending;
  sending = true;
  size_t amount_to_send = data_size;


  //sending the header


  while (sending) {

      if (amount_to_send == 0) {
	  break;
      }

      size_t bytes_sending = (MAX_DATA_SIZE < amount_to_send) ? MAX_DATA_SIZE : amount_to_send;

      SPI_Packet current_packet;
      current_packet.header = bytes_sending;

      // Copy data into packet
      memcpy(current_packet.data, &data[data_size - amount_to_send], current_packet.header * sizeof(uint16_t));
      amount_to_send -= bytes_sending;
      while (!(SPI1->SR & SPI_SR_TXE)) {};  //making sure previous transmission has completed
      SPI1->DR = current_packet.header;


      for (uint16_t counter = 0; counter < current_packet.header; counter++) {
	  while (!(SPI1->SR & SPI_SR_TXE)) {};  //making sure previous transmission has completed
	  SPI1->DR = current_packet.data[counter];
      }

  }

  while (!(SPI1->SR & SPI_SR_TXE)) {};  //making sure previous transmission has completed
  SPI1->DR = 0x0000;

  while (SPI1->SR & SPI_SR_BSY); //exit when busy flag goes low

}





void DAC_write(uint16_t dac_value) {
	  uint16_t data = (dac_value | 0x1000);
	  // while no status register flags and TxFIFO is empty
	  while (!(SPI1->SR & SPI_SR_TXE)) {};
	  SPI1->DR = data;
}



