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
#include "timer.h"
#include <stdio.h>
#include "utils.h"
#include "pp.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t x_cord;
uint8_t y_cord;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  __enable_irq();                          // enable global interrupts)
  LPUART_Init();

  //SPI_Slave_Init();
  //SPI_Master_Init();
  UART3_init();
  init_timer();
  init_user_button();


  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t hclk   = HAL_RCC_GetHCLKFreq();
  uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
  uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();
  char buf[164];
  sprintf(buf, "SYSCLK = %lu Hz\r\nHCLK = %lu Hz\r\nPCLK1 = %lu Hz\r\nPCLK2 = %lu Hz\r\n", sysclk, hclk, pclk1, pclk2);
//  LPUART_Print(buf);

  uint8_t mode = 2; // 0 for single board, 1 for MCU1, 2 for MCU2
  // debug baud
  init_nibble_protocol();
  while(1){
	  pp_MCU1_main();
	  HAL_Delay(50);
  }

//  while(1){
//	  switch(mode){
//	  	  case 0:
//	  		  if(is_pressed()){
//	  			  LPUART_Print("starting op\r\n");
//	  			  UART3_mcu1_solo_matrix();
//	  		  }
//	  		  break;
//	  	  case 1:
//	  		  if(is_pressed()){
//	  			  LPUART_Print("starting op\r\n");
//				  UART3_mcu1_matrix();
//			  }
//	  		  break;
//	  	  case 2:
//	  		  UART3_mcu2_matrix();
//	  		  break;
//	  	  default:
//	  		  break;
//	  }
//  }
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
//  }
  /* USER CODE END 3 */

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
