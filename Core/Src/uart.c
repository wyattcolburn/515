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
    uint8_t charRecv;
    if (USART3->ISR & USART_ISR_RXNE) {
        charRecv = USART3->RDR;
        switch (charRecv) {
            case '\r':
                // If you want to send through USART3 instead of LPUART
                USART3_TransmitString("Enter\r\n");
                break;
            default:
                // Echo character back through USART3
                while(!(USART3->ISR & USART_ISR_TXE));
                USART3->TDR = charRecv;
                break;
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
