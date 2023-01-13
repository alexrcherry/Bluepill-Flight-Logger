/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
 SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t Rx_data[1] = {0};  //  creating a buffer of 10 bytes for uart
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void led(uint8_t val);
void enable_alt(void);
void disable_alt(void);
void read_prom(uint16_t *prom_array);
void read_temp_press(uint16_t *prom, int32_t *temp_press);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  uint16_t prom[8] = {0};
//  read_prom(prom);
//  int32_t T_P[2] = {0};
//  read_temp_press(prom, T_P);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	//check for $ symbol
	HAL_UART_Receive(&huart1, Rx_data, 1, 300);
	if (Rx_data[0] == 36){
		//check if it is a GPGLL NMEA sentence
		uint8_t NMEA[5] = {0};
		HAL_UART_Receive(&huart1, NMEA, 5, 300);
		if (NMEA[4] == 76){
			//convert from DDmm.mm to standard format
			//get latitude
			uint8_t chars[13] = {0};
			HAL_UART_Receive(&huart1, chars, 13, 300);
			uint8_t DD_lat = (chars[1]-48)*10+(chars[2]-48);
			uint32_t mm_lat = ((chars[3]-48)*pow(10, 6)+(chars[4]-48)*pow(10, 5)+(chars[6]-48)*pow(10, 4)+(chars[7]-48)*pow(10, 3)+(chars[8]-48)*pow(10, 2)+(chars[9]-48)*10+(chars[10]-48))/60;
			//latitude direction
			uint8_t dir_lat = chars[12];
			//convert from DDDmm.mm to standard format
			uint8_t lon[14] = {0};
			HAL_UART_Receive(&huart1, lon, 14, 300);
			uint16_t DD_lon = (lon[1]-48)*100+(lon[2]-48)*10+(lon[3]-48);
			uint32_t mm_lon = ((lon[4]-48)*pow(10, 6)+(lon[5]-48)*pow(10, 5)+(lon[7]-48)*pow(10, 4)+(lon[8]-48)*pow(10, 3)+(lon[9]-48)*pow(10, 2)+(lon[10]-48)*10+(lon[11]-48))/60;
			uint8_t dir_lon = lon[13];
			led(1);


		}
	}

	led(0);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void led(uint8_t val){
	if (val == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	}
}

void enable_alt(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
}

void disable_alt(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
}

//read prom
void read_prom(uint16_t *prom_array){
	//reset altimiter to load prom values
	enable_alt();
	uint8_t reset_cmd = 0x1E;
	HAL_SPI_Transmit(&hspi1, &reset_cmd, 1, 10);
	HAL_Delay(5);
	disable_alt();

	uint16_t value = 0;

	for (uint8_t i = 0; i < 8; ++i){
		uint8_t address = (0xA0 | ((i) << 1));
		value = 0;

		enable_alt();
		HAL_SPI_Transmit(&hspi1, &address, 1, 10);
		HAL_SPI_Receive(&hspi1, &value, 2, 10);
		disable_alt();

		prom_array[i] = value;
	}
}

//read compensated pressure
void read_temp_press(uint16_t *prom, int32_t *temp_press){
	uint8_t D1_OSR_256 = 0x40;
	uint8_t read_ADC = 0x00;
	//read pressure
	uint32_t D1 = 0;
	uint8_t uncomp_pres[3] = {0};
	enable_alt();
	HAL_SPI_Transmit(&hspi1, &D1_OSR_256, 1, 10);
	HAL_Delay(1);
	disable_alt();

	enable_alt();
	HAL_SPI_Transmit(&hspi1, &read_ADC, 1, 10);
	HAL_SPI_Receive(&hspi1, &uncomp_pres, 3, 10);
	disable_alt();

	D1 = ((uint32_t) uncomp_pres[0] << 16) | ((uint32_t) uncomp_pres[1] << 8) | (uint32_t) uncomp_pres[2];

	//read temp
	uint32_t D2 = 0;
	uint8_t D2_OSR_256 = 0x50;
	uint8_t uncomp_temp[3] = {0};
	enable_alt();
	HAL_SPI_Transmit(&hspi1, &D2_OSR_256, 1, 10);
	HAL_Delay(1);
	disable_alt();

	enable_alt();
	HAL_SPI_Transmit(&hspi1, &read_ADC, 1, 10);
	HAL_SPI_Receive(&hspi1, &uncomp_temp, 3, 10);
	disable_alt();

	D2 = ((uint32_t) uncomp_temp[0] << 16) | ((uint32_t) uncomp_temp[1] << 8) | (uint32_t) uncomp_temp[2];

	//compensation
	int32_t dT = D2 - prom[4]*pow(2,8);
	int32_t TEMP = 2000+dT*prom[5]/pow(2,23);
	int64_t OFF = prom[1]*pow(2,17)+(prom[3]*dT)/pow(2,6);
	int64_t SENS = prom[0]*pow(2,16)+(prom[2]*dT)/pow(2,7);
	int32_t P = (D1*SENS/pow(2,21)-OFF)/pow(2,15);

	temp_press[0] = TEMP;
	temp_press[1] = P;
}



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
