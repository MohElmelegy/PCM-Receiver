/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/***************************************************Error Detection **************************************************************/
#define lactch_mode1_High_Error_Detection HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define lactch_mode1_Low_Error_Detection HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

/************************************************************************************************************************************/

/***************************************************Serial to paraller**************************************************************/
#define lactch_mode2_High_SerialToParaller HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET)
#define lactch_mode2_Low_SerialToParaller HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET)

/************************************************************************************************************************************/

/***************************************************Data Latch*********************************************************************/
#define lactch_mode4_High_DatatLatch HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define lactch_mode4_Low_DatatLatch HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
/************************************************************************************************************************************/

/***************************************************Latch Clear*********************************************************************/
#define lactch_clear_High HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)
#define lactch_clear_Low HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
/************************************************************************************************************************************/

/***************************************************Data Latch Pin*********************************************************************/
#define lactch_pin_High HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)
#define lactch_pin_Low HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
/************************************************************************************************************************************/

/*************************************************** Address **************************************************************************/
#define lactch_Address1_High HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define lactch_Address1_Low HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)

#define lactch_Address2_High HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define lactch_Address2_Low HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)

#define lactch_Address3_High HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
#define lactch_Address3_Low HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
/***************************************************************************************************************************************/

/***************************************************Mux1 Config **************************************************************************/
#define mux1_Address1_High HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define mux1_Address1_Low HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)

#define mux1_Address2_High HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define mux1_Address2_Low HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)

#define mux1_Address3_High HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define mux1_Address3_Low HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

#define mux1_Enable HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define mux1_disable HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

#define mux1_selection_out HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)

/***************************************************************************************************************************************/

/*********************Error check state***********************************************/
#define disaply_erorr_check 0
#define odd_parity 1
#define even_parity 2
#define Hamming 3
/**********************************************************************************************/

/*********************LED state**************************************************************/
#define LED_OFF 0
#define LED_ON 1
/*******************************************************************************************/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
int status = disaply_erorr_check;
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
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM2_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */
	HAL_Delay(100);

	lactch_pin_High;
	clear();
	HAL_Delay(1);

	// To Enable Mux 1
	mux1_Enable;

	// Init_latch();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		int test_data = 0b0000111;

		status = Error_check();
		// Serial_to_paraller (0b1110000);
		if (status == odd_parity)
		{
			Serial_to_paraller(test_data);
			HAL_Delay(1);
			clear();
			HAL_Delay(1);
			Data_Latch(test_data);
			HAL_Delay(1);
			clear();
			HAL_Delay(1);
			HAL_Delay(1);
			Error_Detec_corr(test_data);
			HAL_Delay(1);
			HAL_Delay(1);
		}

		HAL_Delay(1000);
		clear();
		// Serial_to_paraller (0b0000011);
		HAL_Delay(1000);
		// clear();
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 2700;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 6;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 270;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 270;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	/*Configure GPIO pins : PC13 PC14 PC15 PC0
							 PC1 PC2 PC3 PC4
							 PC5 PC10 PC11 PC12 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA2 PA3
							 PA4 PA5 PA6 PA7
							 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB2 PB10 PB12
							 PB14 PB3 PB4 PB5
							 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB13 PB15 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PD2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/***************************Init latch*************************************/
void Init_latch()
{
	lactch_clear_Low;
	HAL_Delay(1000);
	lactch_pin_High;
	HAL_Delay(10);
	lactch_mode2_High_SerialToParaller;
	lactch_mode4_High_DatatLatch;
	lactch_mode1_High_Error_Detection;
	HAL_Delay(10);
	// Active
	HAL_Delay(1);
	lactch_clear_High;
	HAL_Delay(1);
}
/************************************************************************************/

/***************************Clear latch*************************************/
void clear(void)
{
	lactch_mode2_High_SerialToParaller;
	lactch_mode1_High_Error_Detection;
	lactch_mode4_High_DatatLatch;
	lactch_clear_Low;
}
/**********************************************************************************************************************/

/******************************Addressable latch mode in serial to paraller***********************************/

void latch_data_SerToPara(void)
{
	lactch_mode2_Low_SerialToParaller;
	lactch_mode1_High_Error_Detection;
	lactch_mode4_High_DatatLatch;
	lactch_clear_High;
}
/**********************************************************************************************************************/

/******************************Memory mode in serial to paraller***********************************/

void memory_data_SerToPara(void)
{
	lactch_mode2_High_SerialToParaller;
	lactch_mode1_High_Error_Detection;
	lactch_mode4_High_DatatLatch;
	lactch_clear_High;
}
/**********************************************************************************************************************/

void Serial_to_paraller(int serial_data)
{

	int i = 0;

	for (; i <= 6; i++)
	{
		if (((serial_data >> i) & 1) == 1)
		{
			serial_led(i, LED_ON);
		}
		else
		{
			// No thing
		}
	}
	HAL_Delay(1000);
}

void serial_led(int number, int state)
{
	// if (Error_check_state == 1)

	if (state == 1)
	{
		lactch_pin_High;
		HAL_Delay(1);
		switch (number)
		{
		case 0:
			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_Low;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		case 1:
			lactch_Address1_High;
			lactch_Address2_Low;
			lactch_Address3_Low;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		case 2:
			lactch_Address1_Low;
			lactch_Address2_High;
			lactch_Address3_Low;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		case 3:
			lactch_Address1_High;
			lactch_Address2_High;
			lactch_Address3_Low;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		case 4:
			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_High;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		case 5:
			lactch_Address1_High;
			lactch_Address2_Low;
			lactch_Address3_High;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		case 6:
			lactch_Address1_Low;
			lactch_Address2_High;
			lactch_Address3_High;
			latch_data_SerToPara();
			HAL_Delay(1);
			memory_data_SerToPara();
			HAL_Delay(1);
			break;
		}
	}

	else
	{
		// No thing
	}
}

/***************************Data latch output*********************************************/
void Data_Latch(int serial_data)
{
	static int counter = 0;
	static int syn_counter = 0;
	static int i = 0;
	int syn_code[4] = {1, 0, 1, 0};
	int data[7];
	while (counter <= 7)
	{
		data[counter] = (serial_data >> counter) & 1;

		// To display on leds
		Data_Lactch_led(counter, data[counter]);

		counter++;
	}
	counter = 0;
	HAL_Delay(1000);
}

void Data_Lactch_led(int number, int state)
{
	// if (Error_check_state == 1)

	if (state == 1)
	{
		switch (number)
		{
		case 0:
			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_Low;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		case 1:
			lactch_Address1_High;
			lactch_Address2_Low;
			lactch_Address3_Low;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		case 2:
			lactch_Address1_Low;
			lactch_Address2_High;
			lactch_Address3_Low;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		case 3:
			lactch_Address1_High;
			lactch_Address2_High;
			lactch_Address3_Low;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		case 4:
			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_High;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		case 5:
			lactch_Address1_High;
			lactch_Address2_Low;
			lactch_Address3_High;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		case 6:
			lactch_Address1_Low;
			lactch_Address2_High;
			lactch_Address3_High;
			latch_data_Data_Lactch();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		}
	}

	else
	{
		// No thing
	}
}

void latch_data_Data_Lactch(void)
{
	lactch_mode2_High_SerialToParaller;
	lactch_mode1_High_Error_Detection;
	lactch_mode4_Low_DatatLatch;
	lactch_clear_High;
}

/******************************Memory mode in Data Latch***********************************/

void memory_Data_Latch(void)
{
	lactch_mode2_High_SerialToParaller;
	lactch_mode1_High_Error_Detection;
	lactch_mode4_High_DatatLatch;
	lactch_clear_High;
}
/**************************************************************************************************************/

/**************************************************************************************************************/

/******************************************Error Correction & detection output*********************************/

void Error_Detec_corr(int serial_data)
{
	static int counter = 0;
	static int syn_counter = 0;
	static int i = 0;
	int syn_code[4] = {1, 0, 1, 0};
	int data[7];
	while (counter <= 8)
	{
		data[counter] = (serial_data >> counter) & 1;

		/*To display on leds*/
		Error_Detect_led(counter, data[counter]);

		counter++;
	}
	counter = 0;

	Num_ones(serial_data);
	HAL_Delay(1000);
}

void Error_Detect_led(int number, int state)
{
	// if (Error_check_state == 1)
	// To count ones in data
	static int counter_ones = 0;
	// To compelet 7 times
	static int counter = 0;

	if (state == 1)
	{
		switch (number)
		{
		// H0
		case 0:
			lactch_Address1_High;
			lactch_Address2_Low;
			lactch_Address3_High;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		// H1
		case 1:
			lactch_Address1_Low;
			lactch_Address2_High;
			lactch_Address3_High;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		// H2
		case 2:
			lactch_Address1_High;
			lactch_Address2_High;
			lactch_Address3_High;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		// D3
		case 3:
			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_Low;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		// D4
		case 4:
			lactch_Address1_High;
			lactch_Address2_Low;
			lactch_Address3_Low;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		// D5
		case 5:
			lactch_Address1_Low;
			lactch_Address2_High;
			lactch_Address3_Low;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		// D6
		case 6:
			lactch_Address1_High;
			lactch_Address2_High;
			lactch_Address3_Low;
			counter_ones++;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
			break;
		}
	}

	else
	{
		// No thing
	}
}

void latch_data_Error_Detec(void)
{
	lactch_mode2_High_SerialToParaller;
	lactch_mode1_Low_Error_Detection;
	lactch_mode4_High_DatatLatch;
	lactch_clear_High;
}

void Num_ones(int data)
{
	int counter = 0;
	int i = 0;

	for (; i < 7; i++)
	{
		if (((data >> i) & 1) == 1)
		{

			counter++;
		}
		else
		{
			/*No thing*/
		}
	}

	if (counter % 2 == 0)
	{
		if (status == odd_parity)
		{

			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_High;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
		}
	}
	else
	{
		if (status == even_parity)
		{
			lactch_Address1_Low;
			lactch_Address2_Low;
			lactch_Address3_High;
			latch_data_Error_Detec();
			HAL_Delay(1);
			memory_Data_Latch();
			HAL_Delay(1);
		}
	}
}

/*****************************************************************************************************************************************/

/***************************************************************************************************************************************/
// To check Error detection type
int Error_check(void)
{
	uint8_t error_check = check_Error_switch(4);
	uint8_t odd_ckeck = check_Error_switch(7);
	uint8_t even_check = check_Error_switch(6);
	uint8_t hamming_check = check_Error_switch(5);
	uint8_t state = 0;

	// In case odd parity
	if ((error_check == 1) && (odd_ckeck == 1))
	{
		state = odd_parity;
	}
	// In case even parity
	else if ((error_check == 1) && (even_check == 1))
	{
		state = even_parity;
	}
	// In case hamming
	else if ((error_check == 1) && (hamming_check == 1))
	{
		state = Hamming;
	}
	else
	{
		state = disaply_erorr_check;
	}
	return state;
}

/***************************************************************************************************************************************/

int check_Error_switch(int address)
{
	int arr[3] = {0};
	for (int i = 0; i < 3; i++)
	{
		if (((address >> i) & 1) == 1)
		{
			Mux1_address(i, 1);
		}
		else
		{
			Mux1_address(i, 0);
		}
	}

	return mux1_selection_out;
}

void Mux1_address(int number, int state)
{
	if (state == 1)
	{
		switch (number)
		{
		case 0:
			mux1_Address1_High;
			break;
		case 1:
			mux1_Address2_High;
			break;
		case 2:
			mux1_Address3_High;
			break;
		}
	}
	else if (state == 0)
	{
		switch (number)
		{
		case 0:
			mux1_Address1_Low;
			break;
		case 1:
			mux1_Address2_Low;
			break;
		case 2:
			mux1_Address3_Low;
			break;
		}
	}

	else
	{
		// No thing
	}
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

#ifdef USE_FULL_ASSERT
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
