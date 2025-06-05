
/*
 *		NIBBLES
 *			PD3 - bit 0
 *			PD4 - bit 1
 *			PD5 - bit 2
 *			PD6 - bit 3
 *
 *		CLK
 *			PC6 - custom clock
 *
 *		EN
 *			PD0 - EN (says that whoever set signal high is sending data)
 *
 *			* Note. when PD0 is LOW, NIBBLE GPIO pins must already be able be able to receive data!
 */
#include "main.h"
#include <stdint.h>

#define GPIO_INPUT (0)
#define GPIO_OUTPUT (1)

void set_nibble_mode(uint8_t mode)
{
    /*
     * Sets all Nibble GPIO Pins to mode
     * 	PD3 - bit 0
     * 	PD4 - bit 1
     * 	PD5 - bit 2
     * 	PD6 - bit 3
     *
     * 	Input mode 0x00
     * 	Output mode 0x01
     */
    // Clear values
    GPIOD->MODER &= ~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6);

    if (mode == GPIO_OUTPUT) {
    	// set moder to output
        GPIOD->MODER |= (GPIO_MODER_MODE3_0 | GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0 | GPIO_MODER_MODE6_0);
    }
    // Note clearing the bit sets it to 0x00 which is input mode
    return;
}


void set_EN_mode(uint8_t mode){
	/*
	 * Sets the EN gpio MODER to mode
	 */
	// Sets it to 0x00 - input
    GPIOD->MODER &= ~(GPIO_MODER_MODE0);
    // sets it to 0x01 - ouptut
    if(mode == GPIO_OUTPUT){
    	GPIOD->MODER |= GPIO_MODER_MODE0_0;
    }
}

void set_EN_val(uint8_t on_or_off){
	// 1 for on, 0 for off
    GPIOD->ODR &= ~GPIO_ODR_OD0;
	if(on_or_off){
		GPIOD->ODR |= GPIO_ODR_OD0;
	}
}

void initialize_clk(void)
{
    // --- Enable TIM3 clock ---
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    // Enable GPIOC
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // --- Configure PC6 as Alternate Function AF1 (TIM3_CH1)
    // Set MODER to Alternate (10)
    GPIOC->MODER &= ~(GPIO_MODER_MODE6);
    GPIOC->MODER |= GPIO_MODER_MODE6_1;

    // Set OTYPER to Push-Pull
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT6;

    // Set OSPEEDR to Medium (01)
    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6);
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED6_0;

    // Set PUPDR to No Pull (00)
    GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR6);

    // Set AF2 on PC6
    GPIOC->AFR[0] &= ~(GPIO_AFRL_AFSEL6);
    GPIOC->AFR[0] |= GPIO_AFRL_AFSEL6_1; // AF2 -> TIM3_CH1

    // --- Configure TIM3 for 1 MHz PWM on CH1 ---
    TIM3->PSC = 0;      // No prescaler → 4 MHz timer clock
    TIM3->ARR = 3;      // Period = 4 ticks → 1 us → 1 MHz
    TIM3->CCR1 = 2;     // 50% duty cycle → HIGH for 2 ticks

    // Set PWM Mode 1 on CH1
    TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M);
    TIM3->CCMR1 |= (6 << TIM_CCMR1_OC1M_Pos);  // OC1M = 110 → PWM Mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;            // Enable preload

    // Enable CH1 output
    TIM3->CCER |= TIM_CCER_CC1E;

    // Enable auto-reload preload
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Force update to load registers
    TIM3->EGR |= TIM_EGR_UG;

    // --- DO NOT ENABLE ANY INTERRUPTS! ---
    TIM3->DIER = 0;   // No interrupts → pure hardware PWM

    // READY — do not start TIM3 yet! You can start/stop as needed:
//    TIM3->CR1 |= TIM_CR1_CEN;    // Start
    // TIM3->CR1 &= ~TIM_CR1_CEN;   // Stop
}


void init_nibble_protocol(void){
	// GPIOD
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;

    // set nibble GPIO to input
    set_nibble_mode(GPIO_INPUT);

    // set EN mode to input
    set_EN_mode(GPIO_INPUT);

    // set all gpio to OTYPE = 0x0 (push-pull)
    GPIOD->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT6);

    // set all gpio to OSPEED = 0x01 (medium)

    // clear first
    GPIOD->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED6);
    // set to 0x01
    GPIOD->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_0 | GPIO_OSPEEDR_OSPEED3_0 | GPIO_OSPEEDR_OSPEED4_0 | GPIO_OSPEEDR_OSPEED5_0 | GPIO_OSPEEDR_OSPEED6_0);

    // Set PUPDR to 0x00
    GPIOD->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR4 | GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6);


    initialize_clk();
}

void write_nibble(uint8_t nibble)
{
    // Set nibble on PD3..PD6
    // Clear bits first
    GPIOD->ODR &= ~(GPIO_ODR_OD3 | GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);
    GPIOD->ODR |= (uint32_t) (nibble & 0x0F) << 3;

    // Wait for rising edge → TIM3 UIF flag
    while (!(TIM3->SR & TIM_SR_UIF));

    // Clear UIF flag → ready for next edge
    TIM3->SR &= ~TIM_SR_UIF;
}


void pp_MCU1_main(){
    GPIOD->ODR &= ~(GPIO_ODR_OD3 | GPIO_ODR_OD4 | GPIO_ODR_OD5 | GPIO_ODR_OD6);
	uint8_t A[5] = {0xA, 0xB, 0xC, 0xD, 0xE};
	// set GPIO to ouptut, we are now sending
	set_nibble_mode(GPIO_OUTPUT);
	set_EN_mode(GPIO_OUTPUT);
	set_EN_val(1); // set EN signal high
	TIM3->CR1 |= TIM_CR1_CEN;    // Start
	for (int i = 0; i < sizeof(A) / sizeof(A[0]); i++){
		uint8_t left_nibble, right_nibble;
		left_nibble = (A[i] & 0xF0) >> 4;
		right_nibble = A[i] & 0x0F;
		// write to GPIO before clock edge high.
		write_nibble(left_nibble);
		// wait for next rising edge
		write_nibble(right_nibble);
	}
    TIM3->CR1 &= ~TIM_CR1_CEN;   // Stop
	set_EN_val(0); // set EN signal low
	set_EN_mode(GPIO_INPUT);
	set_nibble_mode(GPIO_INPUT);

}







