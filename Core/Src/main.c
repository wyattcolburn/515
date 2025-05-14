/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "LPUART.h"
#include "uart.h"
#include "spi.h"
#include <string.h>



#define USARTDIV 693 //80 MHz clock, baudrate 115200



/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t x_cord;
uint8_t y_cord;

bool messageReady;
uint16_t SPI_ReceivedData = 0;
bool dataReceived = 0;
//spi values



char UART_MESSAGE[MAX_MESSAGE_SIZE];
uint16_t SPI_REC;

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  LPUART_Init();
  //SPI_Slave_Init();
  //SPI_Master_Init();
  LPUART_Print("hello world");
  UART3_init();
  char* uart_message = "hello franco\r\n";
  uint16_t len = strlen(uart_message);
  uint8_t output_array[len];
  string_to_array_8bit(uart_message, len, output_array);


  while (1) {

      //MCU_1_Main();
      UART_Send_Packet(output_array, len);

      HAL_Delay(1000);

  }
//  while (1) {
//      // Display status of NSS pin
//      if ((GPIOD->IDR & GPIO_IDR_ID0) == 0) {
//          LPUART_Print("NSS is active (LOW)\r\n");
//      } else {
//          LPUART_Print("NSS is inactive (HIGH)\r\n");
//      }
//
//      // Check SPI status register
//      LPUART_Print("SPI2->SR = 0x");
//      print_uint16(SPI2->SR);
//      LPUART_Print("\r\n");
//
//      // Try to read regardless of status
//      SPI_REC = SPI2->DR;
//      LPUART_Print("Read attempt: ");
//      print_uint16(SPI_REC);
//      LPUART_Print("\r\n");
//
//      HAL_Delay(1000);
//  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

void USART2_INIT(void) {
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN); // enable clock for Tx and Rx
    RCC->APB2ENR |= (RCC_APB2ENR_USART1EN); //enable the clock register

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2);
    GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL2_Pos);

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= (0x7 << GPIO_AFRL_AFSEL3_Pos);

    // set OTYPE to PP (0) for both Tx and Rx
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT3);

    // set OSPEED to low (00) for both Tx and Rx
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2);
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3);

    // set PUPD to no resistors (00) for both Tx and Rx
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2);
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3);

    // set MODER to AF
    GPIOA->MODER &= ~(GPIO_MODER_MODE2); 	// clear MODER
    GPIOA->MODER |= GPIO_MODER_MODE2_1;		// set to 10 (AF)

    GPIOA->MODER &= ~(GPIO_MODER_MODE3);	// clear MODER
    GPIOA->MODER |= GPIO_MODER_MODE3_1;		// set to 10 (AF)

    /* END GPIO -------------------------------------------------------------*/

    // Program the M bits in USART_CR1
    USART2->CR1 &= ~(USART_CR1_M1);
    USART2->CR1 &= ~(USART_CR1_M0);

    // set the baud rate
    USART2->BRR = USARTDIV;

    // set stop bits to 1
    USART2->CR2 &= ~(USART_CR2_STOP);	// set to 00 (1 stop bit)]

    // enable the USART
    USART2->CR1 |= (USART_CR1_UE);

    // send an idle frame as first transmission

    USART2->CR1 |= (USART_CR1_TE);

    // enable reception int errupt
    USART2->CR1 |= (USART_CR1_RXNEIE);
    USART2->RQR |= (USART_RQR_RXFRQ);	// clear flag

    // enable the receiver
    USART2->CR1 |= (USART_CR1_RE);

  }


void UART_print(char string[])
{
  int i = 0;
  while (string[i] != '\0')
  {
	while(!(USART2->ISR & USART_ISR_TC)); // wait for TC to be equal to 1
	USART2->TDR = string[i]; 			   // write to UART_TDR
	i++;
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
