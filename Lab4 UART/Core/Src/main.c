/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
volatile int flag;
volatile char currdata; 
volatile char lastdata;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TransChar(char charData);
void TransString(char* stringData);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
	
	// RCC clocks
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Initializing PC10, 11 for UART3 tx, rx
	GPIO_InitTypeDef initStr = {GPIO_PIN_10 | GPIO_PIN_11,
	GPIO_MODE_AF_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	// Initializing LEDs
	GPIO_InitTypeDef ledInitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PB10, PB11
	HAL_GPIO_Init(GPIOC, &ledInitStr); // Initialize pin PC6
	
	// USART3 alternate function setting 
	GPIOC->AFR[1] &= ~(1 << 15);
	GPIOC->AFR[1] &= ~(1 << 14);
	GPIOC->AFR[1] &= ~(1 << 13);
	GPIOC->AFR[1] |= (1 << 12);
	GPIOC->AFR[1] &= ~(1 << 11);	
	GPIOC->AFR[1] &= ~(1 << 10);
	GPIOC->AFR[1] &= ~(1 << 9);
	GPIOC->AFR[1] |= (1 << 8);
	
	
	//Enable USART 3 
	USART3->CR1 |= (1 << 5); // RXNE interrupt enable
	USART3->CR1 |= (1 << 3); // Tx Enable
	USART3->CR1 |= (1 << 2); // Rx Enable
	USART3->CR1 |= (1 << 0); // USART Enable 
	
	USART3->BRR |= HAL_RCC_GetHCLKFreq() / 115200; // get baud rate of 115200

	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
	
//			Exercise 4.1 
//	while(1)
//	{
////		if((USART3->ISR & USART_ISR_RXNE) != 0)
//		{
//			char value = USART3->RDR;
//			switch(value){
//				case 'r':
//					GPIOC->ODR ^= (1 << 6);
//					break;
//				case 'b':
//					GPIOC->ODR ^= (1 << 7);
//					break;
//				case 'o':
//					GPIOC->ODR ^= (1 << 8);
//					break;
//				case 'g':
//					GPIOC->ODR ^= (1 << 9);
//					break;
//			
//				default:
//					TransString("error\n");
//				}		
//	}
	//
	//	}
	
	// initial cmd message
	TransString("CMD?");
	// Initial flag
	flag = 0;
	
	while(1)
	{
				
		if(flag == 1)
		{
			TransChar(currdata); // Display Command Characters
			
				//Turn Off LEDs when 0
				if(currdata== '0'){
					switch(lastdata){
						case 'r':
							GPIOC->ODR &= ~(1 << 6);
							TransString("\nRed off\r\n");
							TransString("CMD?\r\n");
							break;
						case 'b':
							GPIOC->ODR &= ~(1 << 7);
							TransString("\nBlue off\r\n");
							TransString("CMD?\r\n");
							break;
						case 'o':
							GPIOC->ODR &= ~(1 << 8);
							TransString("\nOrange off\r\n");
							TransString("CMD?\r\n");
							break;
						case 'g':
							GPIOC->ODR &= ~(1 << 9);
							TransString("\nGreen off\r\n");
							TransString("CMD?\r\n");
							break;
						
						default: 
							TransString("Wrong Input\r\n");
					}
				}
				// turn on LEDs when 1
				else if(currdata== '1'){
					switch(lastdata)
					{	
						case 'r':
							GPIOC->ODR |= (1 << 6);
							TransString("\nRed on\r\n");
							TransString("CMD?\r\n");
							break;
						case 'b':
							GPIOC->ODR |= (1 << 7);
							TransString("\nBlue on\r\n");
							TransString("CMD?\r\n");
							break;
						case 'o':
							GPIOC->ODR |= (1 << 8);
							TransString("\nOrange on\r\n");
							TransString("CMD?\r\n");
							break;
						case 'g':
							GPIOC->ODR |= (1 << 9);
							TransString("\nGreen on\r\n");
							TransString("CMD?\r\n");
							break;
						
						default: 
							TransString("Wrong Input\r\n");
					}
				
				}
				// Toggle LEDs
				else if(currdata== '2'){

					switch(lastdata)
					{	
						case 'r':
							GPIOC->ODR ^= (1 << 6);
							TransString("\nRed toggle\r\n");
							TransString("CMD?\r\n");
							break;
						case 'b':
							GPIOC->ODR ^= (1 << 7);
							TransString("\nBlue toggle\r\n");
							TransString("CMD?\r\n");
							break;
						case 'o':
							GPIOC->ODR ^= (1 << 8);
							TransString("\nOrange toggle\r\n");
							TransString("CMD?\r\n");
							break;
						case 'g':
							GPIOC->ODR ^= (1 << 9);
							TransString("\nGreen toggle \r\n");
							TransString("CMD?\r\n");
							break;
				
						default: 
							TransString("Wrong Input\r\n");
					}
				}
				else{ 
						if((currdata != 'r' & currdata != 'g' & currdata != 'b' & currdata != 'o')){
						TransString("Wrong Input\r\n");
						}
				}
			}
			flag = 0;
		}
}


// transmit charchacter
void TransChar(char charData)
{
	while (1)
  {
	
		//extract flag, check transmission end flag. Send data when after transmission. 
		if(USART3->ISR & USART_ISR_TXE) 	
		{
			USART3->TDR = charData;
			break;
		}
  }
}
/**
*   transmit string
*/
void TransString(char* stringData)
{
	//parse TransChar function within the stringData
	for (int i = 0; stringData[i] != '\0'; i++) {
			TransChar(stringData[i]);
	}
		
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

// IRQ handler for uart3
void USART3_4_IRQHandler(void)
{
	//store last data for exercise 3
	lastdata = currdata;
	currdata = USART3->RDR;
	//Set flag 
	flag = 1; 
	
}


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