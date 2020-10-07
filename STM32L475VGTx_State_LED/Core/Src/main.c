/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>


//uint16_t device_address=0xBE;
uint16_t register_address = 0x0F;
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t TMP102_ADDR = 0x5F<<1; //use 8bit address
static const uint8_t TMP102_ADDR_RD = 0x5F<<1; //use 8bit address
static const uint8_t REG_TEMP = 0x00;
static const uint8_t T0_degC_x8 = 0x32;
static const uint8_t T1_degC_x8 = 0x33;
static const uint8_t T_MSB = 0x35;
static const uint8_t T1_OUT_1 = 0x3E;
static const uint8_t T1_OUT_2 = 0x3F;
static const uint8_t T0_OUT_1 = 0x3C;
static const uint8_t T0_OUT_2 = 0x3D;
static const uint8_t T_OUT_1 = 0x2A;
static const uint8_t T_OUT_2 = 0x2B;
//buf[7] = {T0_degC_x8,T1_degC_x8, T0_degC, T0_OUT_1, T0_OUT_2, T1_OUT_1, T1_OUT_2 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
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
	HAL_StatusTypeDef ret;
	uint8_t buf[50];
	uint16_t data[50];
	int16_t val, val1;
	float temp_c;
	//uint8_t T0_0, T0_1, T0, T1, T1_0, T1_1;
	int16_t T0_out, T1_out, T_out;
	uint16_t T0_degC_x8_u16, T1_degC_x8_u16;
	//int16_t T0_degC, T1_degC;
	uint8_t buffer[4], tmp, T__MSB;
	int32_t tmp32;
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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //*****************Introduce yourself******************************
	  //buf[0] = {T0_degC_x8};
	 // uint8_t buf[12] = {T0_degC_x8,T1_degC_x8, T0_degC, T0_OUT_1, T0_OUT_2, T1_OUT_1, T1_OUT_2 };
	  buf[0]=register_address;
	  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
	  if(ret != HAL_OK){
		  strcpy((char*)buf, "Error Tx \r\n");
	  }else{
		  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
		  //Read 2 bytes from the Temperature Register
		  if(ret !=HAL_OK){
			  strcpy((char*)buf, "Error Rx \r\n");
		  }
	  }
	  //strcpy((char *)buf, data)
	  sprintf((char*)buf, "Name is: %d\r\n", buf[0]);
	  //Send out buffer (Temperature or error message)
	  HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 500);
	  //Wait

	  //******************Read the temperature High************************
	  buf[0]=T_OUT_2;
	  	  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
	  	  if(ret != HAL_OK){
	  		  strcpy((char*)buf, "Error Tx \r\n");
	  	  }else{
	  		  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
	  		  //Read 2 bytes from the Temperature Register
	  		  if(ret !=HAL_OK){
	  			  strcpy((char*)buf, "Error Rx \r\n");
	  		  }
	  	  }
	  	T_out = ((int16_t) buf[0]);
	  	  //strcpy((char *)buf, data)
	  	  sprintf((char*)buf, "Temperature is: %d H \r\n", T_out<< 8);
	  	  //Send out buffer (Temperature or error message)
	  	  HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 500);
	  	//******************Read the temperature Low************************
	  		  buf[0]=T_OUT_1;
	  		  	  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
	  		  	  if(ret != HAL_OK){
	  		  		  strcpy((char*)buf, "Error Tx \r\n");
	  		  	  }else{
	  		  		  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
	  		  		  //Read 2 bytes from the Temperature Register
	  		  		  if(ret !=HAL_OK){
	  		  			  strcpy((char*)buf, "Error Rx \r\n");
	  		  		  }
	  		  	  }
	  		  	  //strcpy((char *)buf, data)
	  		  	  T_out = (T_out<<8) | buf[0];
	  		  	  sprintf((char*)data, "Temperature is: %d **\r\n", T_out);

	  		  	  //Send out buffer (Temperature or error message)
	  		  	  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);
	//******************Read the T0_degC_x8 Low************************
		  buf[0]=T0_degC_x8;
			  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
			  if(ret != HAL_OK){
				  strcpy((char*)buf, "Error Tx \r\n");
			  }else{
				  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
				  //Read 2 bytes from the Temperature Register
				  if(ret !=HAL_OK){
					  strcpy((char*)buf, "Error Rx \r\n");
				  }
			  }
			  T0_degC_x8_u16 = ((uint16_t) buf[0]); //Convert to 16 bits awaiting the MSB
			  //strcpy((char *)buf, data)
			  sprintf((char*)data, "T0_degC_x8 is: %u  L\r\n", T0_degC_x8_u16);
			  //Send out buffer (Temperature or error message)
			  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);
	  //******************Read the T1_degC_x8 High************************
			  buf[0]=T1_degC_x8;
				  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
				  if(ret != HAL_OK){
					  strcpy((char*)buf, "Error Tx \r\n");
				  }else{
					  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
					  //Read 2 bytes from the Temperature Register
					  if(ret !=HAL_OK){
						  strcpy((char*)buf, "Error Rx \r\n");
					  }
				  }
				  T1_degC_x8_u16 = (uint16_t) buf[0];//Convert to 16 bits awaiting the MSB
				  //strcpy((char *)buf, data)
				  sprintf((char*)data, "T1_degC_x8 is: %u  H\r\n", T1_degC_x8_u16);
				  //Send out buffer (Temperature or error message)
				HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);
	  //******************Read the MSB************************
				  buf[0]=T_MSB;
					  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
					  if(ret != HAL_OK){
						  strcpy((char*)buf, "Error Tx \r\n");
					  }else{
						  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
						  //Read 2 bytes from the Temperature Register
						  if(ret !=HAL_OK){
							  strcpy((char*)buf, "Error Rx \r\n");
						  }
					  }

					  T__MSB = ((uint8_t) buf[0]);//Convert to 16 bits awaiting the MSB
					  //strcpy((char *)buf, data)
					  sprintf((char*)data, "MSB is: %u  \r\n", T__MSB<<8);
					  //Send out buffer (Temperature or error message)
					  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);

	// ******** LOW MSB *******************
					  T0_degC_x8_u16 = (T__MSB<<8 & 0x300)|T0_degC_x8_u16; //Convert to 16 bits awaiting the MSB
					  //strcpy((char *)buf, data)
					  sprintf((char*)data, "T0_degC_x16 is: %u  L\r\n", T0_degC_x8_u16);
					  //Send out buffer (Temperature or error message)
					  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);
  // ******** HIgh MSB *******************
					  T1_degC_x8_u16 = (T__MSB<<6 & 0x300)|T1_degC_x8_u16; //Convert to 16 bits awaiting the MSB
					  //strcpy((char *)buf, data)
					  sprintf((char*)data, "T1_degC_x16 is: %u  L\r\n", T1_degC_x8_u16);
					  //Send out buffer (Temperature or error message)
					  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);

  //******************Read the T0_OUT_2************************
	  buf[0]=T0_OUT_2;
		  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
		  if(ret != HAL_OK){
			  strcpy((char*)buf, "Error Tx \r\n");
		  }else{
			  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
			  //Read 2 bytes from the Temperature Register
			  if(ret !=HAL_OK){
				  strcpy((char*)buf, "Error Rx \r\n");
			  }
		  }
		  T0_out = ((int16_t) buf[0]);
		  //strcpy((char *)buf, data)
		  sprintf((char*)buf, "T0_OUT_2: %u H \r\n", T0_out<<8);
		  //Send out buffer (Temperature or error message)
		  HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 500);
	//******************Read the T0_OUT_1************************
		  buf[0]=T0_OUT_1;
			  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
			  if(ret != HAL_OK){
				  strcpy((char*)buf, "Error Tx \r\n");
			  }else{
				  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
				  //Read 2 bytes from the Temperature Register
				  if(ret !=HAL_OK){
					  strcpy((char*)buf, "Error Rx \r\n");
				  }
			  }
			  //strcpy((char *)buf, data)
			  T0_out = (T0_out<<8)| buf[0];
			  //T_out = (T_out<<8) | buf[0];
			  sprintf((char*)data, "T0_OUT: %u **\r\n", T0_out);

			  //Send out buffer (Temperature or error message)
			  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);
	  //******************Read the T1_OUT_2************************
		  buf[0]=T1_OUT_2;
			  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
			  if(ret != HAL_OK){
				  strcpy((char*)buf, "Error Tx \r\n");
			  }else{
				  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
				  //Read 2 bytes from the Temperature Register
				  if(ret !=HAL_OK){
					  strcpy((char*)buf, "Error Rx \r\n");
				  }
			  }
			  T1_out = ((int16_t) buf[0]);
			  //strcpy((char *)buf, data)
			  sprintf((char*)buf, "T1_OUT_2: %u H \r\n", T1_out<<8);
			  //Send out buffer (Temperature or error message)
			  HAL_UART_Transmit(&huart1, buf, strlen((char*)buf), 500);
		//******************Read the T1_OUT_1************************
			  buf[0]=T1_OUT_1;
				  ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, HAL_MAX_DELAY);
				  if(ret != HAL_OK){
					  strcpy((char*)buf, "Error Tx \r\n");
				  }else{
					  ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 1, 1000);
					  //Read 2 bytes from the Temperature Register
					  if(ret !=HAL_OK){
						  strcpy((char*)buf, "Error Rx \r\n");
					  }
				  }
				  //strcpy((char *)buf, data)
				  T1_out = (T1_out<<8)| buf[0];
				  //T_out = (T_out<<8) | buf[0];
				  sprintf((char*)data, "T1_OUT: %u **\r\n", T1_out);

				  //Send out buffer (Temperature or error message)
				  HAL_UART_Transmit(&huart1, (uint8_t*)data, strlen((char*)data), 500);

	  HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00000E14;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
