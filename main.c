/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cube_schematic.h"
#include "maze_schematic.h"
#include "gyrodirection.h"
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void SET_CUBE(char arr[][5][5]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char CUBEARR[5][5][5] = {0,};
char MAZECUBE[8][8][8] = {0.};
int x = 2,y=2,z=2;
uint8_t rx1_data;
uint8_t bb = 0;
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
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  //HAL_UART_Receive_IT(&huart1, &rx1_data[0],sizeof(rx1_data));
  HAL_UART_Receive_IT(&huart5, &rx1_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //SET_CUBE(square1); // tringle1   tringle2    square1   sphere1   star1
	  //SET_CUBE(front_side); // front_up    front_straight   front_side
	  SET_MAZE_CUBE(maze_basic, z, y, x);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* UART5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART5_IRQn);
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8400-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, A4_Pin|A3_Pin|C3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, A2_Pin|C0_Pin|A1_Pin|C1_Pin
                          |A0_Pin|C2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C4_Pin|D4_Pin|E0_Pin|F1_Pin
                          |F0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, B0_Pin|D0_Pin|B1_Pin|D1_Pin
                          |B2_Pin|D2_Pin|B3_Pin|D3_Pin
                          |B4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, E1_Pin|E2_Pin|E3_Pin|E4_Pin
                          |F4_Pin|F3_Pin|F2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : A4_Pin A3_Pin C3_Pin */
  GPIO_InitStruct.Pin = A4_Pin|A3_Pin|C3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : A2_Pin C0_Pin A1_Pin C1_Pin
                           A0_Pin C2_Pin */
  GPIO_InitStruct.Pin = A2_Pin|C0_Pin|A1_Pin|C1_Pin
                          |A0_Pin|C2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C4_Pin D4_Pin E0_Pin F1_Pin
                           F0_Pin */
  GPIO_InitStruct.Pin = C4_Pin|D4_Pin|E0_Pin|F1_Pin
                          |F0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B0_Pin D0_Pin B1_Pin D1_Pin
                           B2_Pin D2_Pin B3_Pin D3_Pin
                           B4_Pin */
  GPIO_InitStruct.Pin = B0_Pin|D0_Pin|B1_Pin|D1_Pin
                          |B2_Pin|D2_Pin|B3_Pin|D3_Pin
                          |B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : E1_Pin E2_Pin E3_Pin E4_Pin
                           F4_Pin F3_Pin F2_Pin */
  GPIO_InitStruct.Pin = E1_Pin|E2_Pin|E3_Pin|E4_Pin
                          |F4_Pin|F3_Pin|F2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{/*
	if(huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(&huart1, rx1_data,1);
		HAL_UART_Transmit(&huart1, &bb, 1, 10);
	}*/

	if(huart->Instance == UART5)
		{/*
			HAL_UART_Receive_IT(&huart5, &rx1_data,1);

			bb = bb + rx1_data - '0';
			HAL_UART_Transmit(&huart5, &bb, 1, 10);*/

			HAL_UART_Receive_IT(&huart5, &rx1_data, 1);

			if(rx1_data == 'r' && x<6)x++;
			else if(rx1_data == 'l' && x>2)x--;
			else if(rx1_data == 'g' && y>2)y--;
			else if(rx1_data == 'b' && y<6)y++;
			else if(rx1_data == 'u' && z<6)z++;
			else if(rx1_data == 'd' && z>2)z--;


		}


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		//HAL_UART_Transmit(&huart5, &a[0], 1, 10);

/*
		CUBEARR[z][x][y] = 0;
		if((z < 4) & (x == 4) & (y == 4) ) z++;  else if( (z == 4) & (x == 4) & (y == 4) ) z = 0;
		if((x < 4) & (y == 4)) x++;  else if((x == 4) & (y == 4) ) x = 0;
		if(y < 4) y++;  else y = 0;
		CUBEARR[z][x][y] = 1;
*/


	}
}

void SET_CUBE(char arr[][5][5]){ // A : z , B~E : x y

	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[0][0][0]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[0][0][1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[0][0][2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[0][0][3]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[0][0][4]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[0][1][0]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[0][1][1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[0][1][2]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[0][1][3]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[0][1][4]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[0][2][0]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[0][2][1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[0][2][2]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[0][2][3]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[0][2][4]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[0][3][0]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[0][3][1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[0][3][2]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[0][3][3]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[0][3][4]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[0][4][0]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[0][4][1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[0][4][2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[0][4][3]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[0][4][4]);
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 1);
	HAL_Delay(2);

	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[1][0][0]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[1][0][1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[1][0][2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[1][0][3]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[1][0][4]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[1][1][0]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[1][1][1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[1][1][2]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[1][1][3]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[1][1][4]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[1][2][0]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[1][2][1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[1][2][2]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[1][2][3]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[1][2][4]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[1][3][0]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[1][3][1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[1][3][2]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[1][3][3]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[1][3][4]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[1][4][0]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[1][4][1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[1][4][2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[1][4][3]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[1][4][4]);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 1);
	HAL_Delay(2);

	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[2][0][0]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[2][0][1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[2][0][2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[2][0][3]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[2][0][4]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[2][1][0]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[2][1][1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[2][1][2]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[2][1][3]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[2][1][4]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[2][2][0]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[2][2][1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[2][2][2]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[2][2][3]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[2][2][4]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[2][3][0]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[2][3][1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[2][3][2]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[2][3][3]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[2][3][4]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[2][4][0]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[2][4][1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[2][4][2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[2][4][3]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[2][4][4]);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 1);
	HAL_Delay(2);

	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[3][0][0]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[3][0][1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[3][0][2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[3][0][3]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[3][0][4]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[3][1][0]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[3][1][1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[3][1][2]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[3][1][3]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[3][1][4]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[3][2][0]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[3][2][1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[3][2][2]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[3][2][3]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[3][2][4]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[3][3][0]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[3][3][1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[3][3][2]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[3][3][3]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[3][3][4]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[3][4][0]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[3][4][1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[3][4][2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[3][4][3]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[3][4][4]);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 1);
	HAL_Delay(2);

	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[4][0][0]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[4][0][1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[4][0][2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[4][0][3]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[4][0][4]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[4][1][0]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[4][1][1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[4][1][2]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[4][1][3]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[4][1][4]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[4][2][0]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[4][2][1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[4][2][2]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[4][2][3]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[4][2][4]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[4][3][0]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[4][3][1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[4][3][2]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[4][3][3]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[4][3][4]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[4][4][0]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[4][4][1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[4][4][2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[4][4][3]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[4][4][4]);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 1);
	HAL_Delay(2);

}

void SET_MAZE_CUBE(char arr[][9][9], int z, int y, int x){ // A : z , B~E : x y

	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z-2][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z-2][y-1][x-2]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z-2][y][x-2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z-2][y+1][x-2]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z-2][y+2][x-2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z-2][y-2][x-1]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z-2][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z-2][y][x-1]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z-2][y+1][x-1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z-2][y+2][x-1]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z-2][y-2][x]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z-2][y-1][x]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z-2][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z-2][y+1][x]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z-2][y+2][x]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z-2][y-2][x+1]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z-2][y-1][x+1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z-2][y][x+1]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z-2][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z-2][y+2][x+1]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z-2][y-2][x+2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z-2][y-1][x+2]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z-2][y][x+2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z-2][y+1][x+2]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z-2][y+2][x+2]);
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z-1][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z-1][y-1][x-2]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z-1][y][x-2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z-1][y+1][x-2]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z-1][y+2][x-2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z-1][y-2][x-1]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z-1][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z-1][y][x-1]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z-1][y+1][x-1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z-1][y+2][x-1]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z-1][y-2][x]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z-1][y-1][x]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z-1][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z-1][y+1][x]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z-1][y+2][x]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z-1][y-2][x+1]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z-1][y-1][x+1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z-1][y][x+1]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z-1][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z-1][y+2][x+1]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z-1][y-2][x+2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z-1][y-1][x+2]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z-1][y][x+2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z-1][y+1][x+2]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z-1][y+2][x+2]);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z][y-1][x-2]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z][y][x-2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z][y+1][x-2]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z][y+2][x-2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z][y-2][x-1]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z][y][x-1]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z][y+1][x-1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z][y+2][x-1]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z][y-2][x]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z][y-1][x]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z][y+1][x]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z][y+2][x]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z][y-2][x+1]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z][y-1][x+1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z][y][x+1]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z][y+2][x+1]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z][y-2][x+2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z][y-1][x+2]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z][y][x+2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z][y+1][x+2]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z][y+2][x+2]);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z+1][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z+1][y-1][x-2]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z+1][y][x-2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z+1][y+1][x-2]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z+1][y+2][x-2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z+1][y-2][x-1]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z+1][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z+1][y][x-1]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z+1][y+1][x-1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z+1][y+2][x-1]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z+1][y-2][x]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z+1][y-1][x]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z+1][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z+1][y+1][x]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z+1][y+2][x]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z+1][y-2][x+1]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z+1][y-1][x+1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z+1][y][x+1]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z+1][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z+1][y+2][x+1]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z+1][y-2][x+2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z+1][y-1][x+2]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z+1][y][x+2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z+1][y+1][x+2]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z+1][y+2][x+2]);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z+2][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z+2][y-1][x-2]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z+2][y][x-2]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z+2][y+1][x-2]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z+2][y+2][x-2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z+2][y-2][x-1]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z+2][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z+2][y][x-1]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z+2][y+1][x-1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z+2][y+2][x-1]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z+2][y-2][x]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z+2][y-1][x]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z+2][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z+2][y+1][x]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z+2][y+2][x]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z+2][y-2][x+1]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z+2][y-1][x+1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z+2][y][x+1]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z+2][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z+2][y+2][x+1]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z+2][y-2][x+2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z+2][y-1][x+2]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z+2][y][x+2]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z+2][y+1][x+2]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z+2][y+2][x+2]);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 1);
	HAL_Delay(1);

/*
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z-2][x-2][y-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z-2][x-2][y-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z-2][x-2][y]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z-2][x-2][y+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z-2][x-2][y+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z-2][x-1][y-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z-2][x-1][y-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z-2][x-1][y]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z-2][x-1][y+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z-2][x-1][y+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z-2][x][y-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z-2][x][y-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z-2][x][y]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z-2][x][y+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z-2][x][y+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z-2][x+1][y-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z-2][x+1][y-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z-2][x+1][y]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z-2][x+1][y+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z-2][x+1][y+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z-2][x+2][y-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z-2][x+2][y-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z-2][x+2][y]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z-2][x+2][y+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z-2][x+2][y+2]);
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z-1][x-2][y-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z-1][x-2][y-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z-1][x-2][y]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z-1][x-2][y+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z-1][x-2][y+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z-1][x-1][y-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z-1][x-1][y-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z-1][x-1][y]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z-1][x-1][y+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z-1][x-1][y+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z-1][x][y-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z-1][x][y-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z-1][x][y]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z-1][x][y+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z-1][x][y+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z-1][x+1][y-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z-1][x+1][y-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z-1][x+1][y]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z-1][x+1][y+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z-1][x+1][y+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z-1][x+2][y-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z-1][x+2][y-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z-1][x+2][y]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z-1][x+2][y+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z-1][x+2][y+2]);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z][x-2][y-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z][x-2][y-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z][x-2][y]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z][x-2][y+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z][x-2][y+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z][x-1][y-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z][x-1][y-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z][x-1][y]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z][x-1][y+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z][x-1][y+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z][x][y-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z][x][y-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z][x][y]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z][x][y+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z][x][y+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z][x+1][y-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z][x+1][y-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z][x+1][y]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z][x+1][y+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z][x+1][y+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z][x+2][y-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z][x+2][y-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z][x+2][y]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z][x+2][y+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z][x+2][y+2]);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z+1][x-2][y-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z+1][x-2][y-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z+1][x-2][y]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z+1][x-2][y+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z+1][x-2][y+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z+1][x-1][y-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z+1][x-1][y-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z+1][x-1][y]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z+1][x-1][y+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z+1][x-1][y+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z+1][x][y-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z+1][x][y-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z+1][x][y]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z+1][x][y+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z+1][x][y+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z+1][x+1][y-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z+1][x+1][y-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z+1][x+1][y]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z+1][x+1][y+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z+1][x+1][y+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z+1][x+2][y-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z+1][x+2][y-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z+1][x+2][y]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z+1][x+2][y+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z+1][x+2][y+2]);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z+2][x-2][y-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z+2][x-2][y-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z+2][x-2][y]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z+2][x-2][y+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z+2][x-2][y+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z+2][x-1][y-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z+2][x-1][y-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z+2][x-1][y]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z+2][x-1][y+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z+2][x-1][y+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z+2][x][y-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z+2][x][y-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z+2][x][y]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z+2][x][y+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z+2][x][y+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z+2][x+1][y-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z+2][x+1][y-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z+2][x+1][y]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z+2][x+1][y+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z+2][x+1][y+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z+2][x+2][y-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z+2][x+2][y-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z+2][x+2][y]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z+2][x+2][y+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z+2][x+2][y+2]);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 1);
	HAL_Delay(1);
*/
/*
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z-2][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z-2][y-2][x-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z-2][y-2][x]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z-2][y-2][x+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z-2][y-2][x+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z-2][y-1][x-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z-2][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z-2][y-1][x]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z-2][y-1][x+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z-2][y-1][x+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z-2][y][x-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z-2][y][x-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z-2][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z-2][y][x+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z-2][y][x+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z-2][y+1][x-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z-2][y+1][x-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z-2][y+1][x]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z-2][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z-2][y+1][x+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z-2][y+2][x-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z-2][y+2][x-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z-2][y+2][x]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z-2][y+2][x+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z-2][y+2][x+2]);
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z-1][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z-1][y-2][x-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z-1][y-2][x]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z-1][y-2][x+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z-1][y-2][x+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z-1][y-1][x-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z-1][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z-1][y-1][x]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z-1][y-1][x+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z-1][y-1][x+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z-1][y][x-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z-1][y][x-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z-1][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z-1][y][x+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z-1][y][x+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z-1][y+1][x-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z-1][y+1][x-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z-1][y+1][x]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z-1][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z-1][y+1][x+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z-1][y+2][x-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z-1][y+2][x-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z-1][y+2][x]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z-1][y+2][x+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z-1][y+2][x+2]);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z][y-2][x-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z][y-2][x]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z][y-2][x+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z][y-2][x+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z][y-1][x-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z][y-1][x]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z][y-1][x+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z][y-1][x+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z][y][x-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z][y][x-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z][y][x+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z][y][x+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z][y+1][x-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z][y+1][x-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z][y+1][x]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z][y+1][x+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z][y+2][x-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z][y+2][x-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z][y+2][x]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z][y+2][x+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z][y+2][x+2]);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z+1][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z+1][y-2][x-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z+1][y-2][x]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z+1][y-2][x+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z+1][y-2][x+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z+1][y-1][x-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z+1][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z+1][y-1][x]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z+1][y-1][x+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z+1][y-1][x+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z+1][y][x-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z+1][y][x-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z+1][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z+1][y][x+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z+1][y][x+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z+1][y+1][x-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z+1][y+1][x-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z+1][y+1][x]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z+1][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z+1][y+1][x+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z+1][y+2][x-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z+1][y+2][x-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z+1][y+2][x]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z+1][y+2][x+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z+1][y+2][x+2]);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 1);
	HAL_Delay(1);

	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, 0);
	HAL_GPIO_WritePin(B0_GPIO_Port, B0_Pin, arr[z+2][y-2][x-2]);
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, arr[z+2][y-2][x-1]);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, arr[z+2][y-2][x]);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, arr[z+2][y-2][x+1]);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, arr[z+2][y-2][x+2]);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, arr[z+2][y-1][x-2]);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, arr[z+2][y-1][x-1]);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, arr[z+2][y-1][x]);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, arr[z+2][y-1][x+1]);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, arr[z+2][y-1][x+2]);
	HAL_GPIO_WritePin(D0_GPIO_Port, D0_Pin, arr[z+2][y][x-2]);
	HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, arr[z+2][y][x-1]);
	HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, arr[z+2][y][x]);
	HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, arr[z+2][y][x+1]);
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, arr[z+2][y][x+2]);
	HAL_GPIO_WritePin(E0_GPIO_Port, E0_Pin, arr[z+2][y+1][x-2]);
	HAL_GPIO_WritePin(E1_GPIO_Port, E1_Pin, arr[z+2][y+1][x-1]);
	HAL_GPIO_WritePin(E2_GPIO_Port, E2_Pin, arr[z+2][y+1][x]);
	HAL_GPIO_WritePin(E3_GPIO_Port, E3_Pin, arr[z+2][y+1][x+1]);
	HAL_GPIO_WritePin(E4_GPIO_Port, E4_Pin, arr[z+2][y+1][x+2]);
	HAL_GPIO_WritePin(F0_GPIO_Port, F0_Pin, arr[z+2][y+2][x-2]);
	HAL_GPIO_WritePin(F1_GPIO_Port, F1_Pin, arr[z+2][y+2][x-1]);
	HAL_GPIO_WritePin(F2_GPIO_Port, F2_Pin, arr[z+2][y+2][x]);
	HAL_GPIO_WritePin(F3_GPIO_Port, F3_Pin, arr[z+2][y+2][x+1]);
	HAL_GPIO_WritePin(F4_GPIO_Port, F4_Pin, arr[z+2][y+2][x+2]);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, 1);
	HAL_Delay(1);
*/

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
