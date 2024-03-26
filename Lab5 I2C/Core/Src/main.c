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
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TSC_HandleTypeDef htsc;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_I2C2_Init(void);
//static void MX_SPI2_Init(void);
//static void MX_TSC_Init(void);
//static void MX_USB_PCD_Init(void);
//void I2CWriteCheck1(void);
//void I2CReadCheck1(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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

  /* USER CODE BEGIN SysInit */

	
	//initalize the RCC clocks for gpio c and b
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	//Initalize I2C clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;




		// Initializing LEDs
	GPIO_InitTypeDef ledInitStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
	GPIO_MODE_OUTPUT_PP,
	GPIO_SPEED_FREQ_LOW,
	GPIO_NOPULL};
	
	HAL_GPIO_Init(GPIOC, &ledInitStr); // Initialize LED pins
	
 // Initialize PB 11, PB 13
  GPIO_InitTypeDef initStr1 = {GPIO_PIN_11 | GPIO_PIN_13,
                               GPIO_MODE_AF_OD,  // AF mode
                               GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOB, &initStr1);
						

  // initialize PB14 
  GPIO_InitTypeDef initStr2 = {GPIO_PIN_14, 
                               GPIO_MODE_OUTPUT_PP,  // OUTPUT mode
                               GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOB, &initStr2);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // set pin8 high

// initialize PB15 to input mode
  GPIO_InitTypeDef initStr3 = {GPIO_PIN_15, 
                               GPIO_MODE_INPUT,  // INPUT mode
                               GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOB, &initStr3);
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // set pin8 high

															  // initialize PC0
  GPIO_InitTypeDef initStr4 = {GPIO_PIN_0,
                               GPIO_MODE_OUTPUT_PP,  // OUTPUT mode
                               GPIO_SPEED_FREQ_LOW, GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr4);
															 
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);  // set pin8 high
															

															 
															 
															 
															 
															 
															 
															 
															 
															 
															 
	// set PB11 and PB 13 to I2C2_SDA and I2C2_SCL 
	// PB11- af1, PB13- AF5
	GPIOB->AFR[1] &= ~(1 << 15);
	GPIOB->AFR[1] &= ~(1 << 14);
	GPIOB->AFR[1] &= ~(1 << 13);
	GPIOB->AFR[1] |= (1 << 12);
															 
	// PB13- AF5														 
	GPIOB->AFR[1] &= ~(1 << 23);	
	GPIOB->AFR[1] |= (1 << 22);
	GPIOB->AFR[1] &= ~(1 << 21);
	GPIOB->AFR[1] |= (1 << 20);													 
		
								
	I2C2->TIMINGR |= (0x1<<28); //presc =1
	// SCLL = 0X13
	
	I2C2->TIMINGR	|= (1<<0);
	I2C2->TIMINGR	|= (1<<1);
	I2C2->TIMINGR	&= ~(1<<2);
	I2C2->TIMINGR	&= ~(1<<3);
	
	I2C2->TIMINGR	|= (1<<4);
	I2C2->TIMINGR	&= ~(1<<5);
	I2C2->TIMINGR	&= ~(1<<6);
	I2C2->TIMINGR	&= ~(1<<7);
	//SCLH = 0XF
	
	I2C2->TIMINGR	|= (1<<8);
	I2C2->TIMINGR	|= (1<<9);
	I2C2->TIMINGR	|= (1<<10);
	I2C2->TIMINGR	|= (1<<11);
	
	I2C2->TIMINGR	&= ~(1<<12);
	I2C2->TIMINGR	&= ~(1<<13);
	I2C2->TIMINGR	&= ~(1<<14);
	I2C2->TIMINGR	&= ~(1<<15);
	//SDADEL = 0X2
	
	I2C2->TIMINGR	&= ~(1<<16);
	I2C2->TIMINGR	|= (1<<17);
	I2C2->TIMINGR	&= ~(1<<18);
	I2C2->TIMINGR	&= ~(1<<19);
	//SCLDEL = 0X4
	I2C2->TIMINGR	&= ~(1<<20);
	I2C2->TIMINGR	&= ~(1<<21);
	I2C2->TIMINGR	|= (1<<22);
	I2C2->TIMINGR	&= ~(1<<23);
	
	I2C2->CR1 |= (1<<0);	//PERIPHERAL ENABLE
	
	// SADD(SLAVE ADDRESS) TO 0X69 = 0110 1001
	
	I2C2->CR2 |= (1<<1);
	I2C2->CR2 &= ~(1<<2);
	I2C2->CR2 &= ~(1<<3);
	I2C2->CR2 |= (1<<4);
	
	I2C2->CR2 &= ~(1<<5);
	I2C2->CR2 |= (1<<6);
	I2C2->CR2 |= (1<<7);

	//SET NUMBER OF BYTES TO TRANSMIT TO 1
//	
	I2C2->CR2 |= (1<<16);
	I2C2->CR2 &= ~(1<<17);
	I2C2->CR2 &= ~(1<<18);
	I2C2->CR2 &= ~(1<<19);
	
	I2C2->CR2 &= ~(1<<20);
	I2C2->CR2 &= ~(1<<21);
	I2C2->CR2 &= ~(1<<22);
	I2C2->CR2 &= ~(1<<23);


// RD_WRN TO WRITE =0
	I2C2->CR2 &= ~(1<<10);
	
	//SET START BIT
	I2C2->CR2 |= (1<<13);
											
  /* USER CODE END 2 */

 
	
//---------------------First Checkoff Starts-----------------------

	/*--------------------WRITE------------------------------------*/
  while (1)
  { 
		if((I2C2->ISR & I2C_ISR_TXIS)){
			break;
		}
	}
	//setting the address of the who-am-i register to 0x0F
	 I2C2->TXDR = 0x0F;
	
	//Wait for TC flag
	 while(1){
		 if((I2C2->ISR & I2C_ISR_TC)){
			 break;
		 }
	 }
		// SADD(SLAVE ADDRESS) TO 0X69 = 0110 1001

	I2C2->CR2 |= (1<<1);
	I2C2->CR2 &= ~(1<<2);
	I2C2->CR2 &= ~(1<<3);
	I2C2->CR2 |= (1<<4);
	
	I2C2->CR2 &= ~(1<<5);
	I2C2->CR2 |= (1<<6);
	I2C2->CR2 |= (1<<7);
			//SET NUMBER OF BYTES TO TRANSMIT TO 1
			I2C2->CR2 |= (0x01<<16);
		

			// RD_WRN TO READ89
			I2C2->CR2 |= (1<<10);
			
			//SET START BIT
			I2C2->CR2 |= (1<<13);
			
	/*------------------------READ---------------------------*/
			
			while(1){
				if((I2C2-> ISR & I2C_ISR_RXNE)){
					break;
				}
			}
			while(1){
				if((I2C2->ISR & I2C_ISR_TC)){
					break;
				}
			}
			  
		if(I2C2->RXDR == 0xD3){
			GPIOC->ODR |= (1 << 9);
		}
		 I2C2->CR2 |= (1 << 14);
}


// ------------------------------First checkoff End----------------------------


////--------------------------Second Checkoff Start--------------------------------
//// address for the gyroscope is 0x69
////enable gyroscope
//// SADD(SLAVE ADDRESS) TO 0X69 = 0110 1001
//	I2C2->CR2|= (0x69<<1);
//	
////	I2C2->CR2 |= (1<<1);
////	I2C2->CR2 &= ~(1<<2);
////	I2C2->CR2 &= ~(1<<3);
////	I2C2->CR2 |= (1<<4);
////	
////	I2C2->CR2 &= ~(1<<5);
////	I2C2->CR2 |= (1<<6);
////	I2C2->CR2 |= (1<<7);

//	//SET NUMBER OF BYTES TO TRANSMIT TO 2
////	
//	I2C2->CR2 &= ~(1<<16);
//	I2C2->CR2 |= (1<<17);


//// RD_WRN TO WRITE =0
//	I2C2->CR2 &= ~(1<<10);
//	
//	//SET START BIT
//	I2C2->CR2 |= (1<<13);
//	
//	/*--------------------Enable Gyroscope - normal mode, enX, enY ------------------------------------*/
//  while (1)
//  { 
//		if((I2C2->ISR & I2C_ISR_TXIS)){
//			break;
//		}
//	}
//	//setting the address of the CTRL_REG1
//	 I2C2->TXDR = 0x20;
//	
//	 while (1)
//  { 
//		if((I2C2->ISR & I2C_ISR_TXIS)){
//			break;
//		}
//	}
//		 I2C2->TXDR = 0xB;		// 1011

//	//Wait for TC flag
//	 while(1){
//		 if((I2C2->ISR & I2C_ISR_TC)){
//			 break;
//		 }
//	 }
//	I2C2->CR2 |= (1 << 14); //stop
//	
//		
//			
//	/*------------------------Read X Data---------------------------*/
//	 
//			//initiate x,y variables
//	 int8_t x1;
//	 int8_t x2;
//	 int8_t y1;
//	 int8_t y2;
//	 int16_t x;
//	 int16_t y;
//	 
//		while(1){
//		HAL_Delay(100);
//				//SET NUMBER OF BYTES TO TRANSMIT TO 1
//		I2C2->CR2 |= (1<<16);
//		I2C2->CR2 &= ~(1<<17);

//			// RD_WRN TO Write
//		I2C2->CR2 &= ~(1<<10);
//			
//			//SET START BIT
//		I2C2->CR2 |= (1<<13);
//		
//				while(1) {
//					if((I2C2-> ISR & I2C_ISR_TXIS)){
//						break;
//					}
//				}
//		I2C2->TXDR = 0xA8; //x data h, l combined
//			
//			while(1){
//				if((I2C2->ISR & I2C_ISR_TC)){
//					break;
//				}
//			}
//			
//				//SET NUMBER OF BYTES TO TRANSMIT TO 2
//		I2C2->CR2 &= ~(1<<16);
//		I2C2->CR2 |= (1<<17);

//			// RD_WRN TO READ (READ=1, WRT=0)
//		I2C2->CR2 |= (1<<10);
//			
//			//SET START BIT 
//		I2C2->CR2 |= (1<<13);
//			
//			while(1) {
//					if((I2C2-> ISR & I2C_ISR_RXNE)){
//						break;
//					}
//				}
//			x1= I2C2->RXDR;
//				
//			while(1) {
//					if((I2C2-> ISR & I2C_ISR_RXNE)){
//						break;
//					}
//			}
//			x2=  I2C2->RXDR;
//			while(1){
//				if((I2C2->ISR & I2C_ISR_TC)){
//					break;
//				}
//			}
//			//stop
//			I2C2->CR2 |= (1<<14);
//			
//			  
////----------------------READ Y DATA----------------------
//				//SET NUMBER OF BYTES TO TRANSMIT TO 1
//		I2C2->CR2 |= (1<<16);
//		I2C2->CR2 &= ~(1<<17);
//			// RD_WRN TO write (READ=1, WRT=0)
//		I2C2->CR2 &= ~(1<<10);
//			//SET START BIT
//		I2C2->CR2 |= (1<<13);
//			
//			// SEND Y ADDRESS
//				while(1) {
//					if((I2C2-> ISR & I2C_ISR_TXIS)){
//						break;
//					}
//				}
//		I2C2->TXDR = 0xAA;	// Y DATA LOW, HIGH COMBINED ADDRESS
//			
//			while(1){
//				if((I2C2->ISR & I2C_ISR_TC)){
//					break;
//				}
//			}
//				//SET NUMBER OF BYTES TO TRANSMIT TO 2
//		I2C2->CR2 &= ~(1<<16);
//		I2C2->CR2 |= (1<<17);

//			// RD_WRN TO READ (READ=1, WRT=0)
//		I2C2->CR2 |= (1<<10);
//			
//			//SET START BIT 
//		I2C2->CR2 |= (1<<13);

//			//RX Y DATA
//	while(1) {
//					if((I2C2-> ISR & I2C_ISR_RXNE)){
//						break;
//					}
//				}
//			y1= I2C2->RXDR;
//				
//			while(1) {
//					if((I2C2-> ISR & I2C_ISR_RXNE)){
//						break;
//					}
//			}
//			y2=  I2C2->RXDR;
//			while(1){
//				if((I2C2->ISR & I2C_ISR_TC)){
//					break;
//				}
//			}
//			// stop transaction
//			I2C2->CR2 |= (1<<14);
//			
//		//--------------------end of x,y data read-----------------------------
//		x = (uint16_t)((x2 << 8) | x1);
//		y = (uint16_t)((y2 << 8) | y1);
//			int threshhold= 130;

//	if (x > threshhold) {	// x,y  value default = +-245 
//		//turn green on, orange off
//			GPIOC->ODR |= (1 << 9);
//		  GPIOC->ODR &= ~(1 << 8);
//	}
//	else if (x < -threshhold){
//			// turn Orange on, green off
//			GPIOC->ODR |= (1 << 8);
//		  GPIOC->ODR &= ~(1 << 9);
//	}
//	
//	else if( y >threshhold){
//		//turn red on, blue off
//			GPIOC->ODR |= (1 << 6);
//		  GPIOC->ODR &= ~(1 << 7);
//	}
//		else if( y < -threshhold){
//			//turn blue on, red off
//			GPIOC->ODR |= (1 << 7);
//		  GPIOC->ODR &= ~(1 << 6);
//	}
//	
//}
		





	

















void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TSC Initialization Function
  * @param None
  * @retval None
  */
static void MX_TSC_Init(void)
{

  /* USER CODE BEGIN TSC_Init 0 */

  /* USER CODE END TSC_Init 0 */

  /* USER CODE BEGIN TSC_Init 1 */

  /* USER CODE END TSC_Init 1 */

  /** Configure the TSC peripheral
  */
  htsc.Instance = TSC;
  htsc.Init.CTPulseHighLength = TSC_CTPH_2CYCLES;
  htsc.Init.CTPulseLowLength = TSC_CTPL_2CYCLES;
  htsc.Init.SpreadSpectrum = DISABLE;
  htsc.Init.SpreadSpectrumDeviation = 1;
  htsc.Init.SpreadSpectrumPrescaler = TSC_SS_PRESC_DIV1;
  htsc.Init.PulseGeneratorPrescaler = TSC_PG_PRESC_DIV4;
  htsc.Init.MaxCountValue = TSC_MCV_8191;
  htsc.Init.IODefaultMode = TSC_IODEF_OUT_PP_LOW;
  htsc.Init.SynchroPinPolarity = TSC_SYNC_POLARITY_FALLING;
  htsc.Init.AcquisitionMode = TSC_ACQ_MODE_NORMAL;
  htsc.Init.MaxCountInterrupt = DISABLE;
  htsc.Init.ChannelIOs = TSC_GROUP1_IO3|TSC_GROUP2_IO3|TSC_GROUP3_IO2;
  htsc.Init.ShieldIOs = 0;
  htsc.Init.SamplingIOs = TSC_GROUP1_IO4|TSC_GROUP2_IO4|TSC_GROUP3_IO3;
  if (HAL_TSC_Init(&htsc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TSC_Init 2 */

  /* USER CODE END TSC_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin EXT_RESET_Pin LD3_Pin LD6_Pin
                           LD4_Pin LD5_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|EXT_RESET_Pin|LD3_Pin|LD6_Pin
                          |LD4_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
