/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "mpu6050.h"
#include <math.h>
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
 I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static uint8_t RData = 0x35, Count=0;
uint32_t Before, After, constant=100;
static float Offset = 0;
static double AngleX=0, AngleTarget, I=0, AngleBefore = 0 ,Output, Kp = 90, Kd =1.5, Ki = 700; 
static MPU6050_t MPU6050;
static float dt = 0.005, Fs;
void GoAhead(void) {
HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);	
HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);	
}
void GoBack(void) {
HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);	
HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}
void TurnLeft(void) {
HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);	
HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}
void TurnRight(void) {
HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);	
HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
}
short ReadEncoder(void) 
		{
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
		Before = __HAL_TIM_GetCounter(&htim1);
		HAL_Delay(10);
		After = __HAL_TIM_GetCounter(&htim1);
		return (After - Before);
		}

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
	Fs = 1000*dt;
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	//HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
	HAL_UART_Receive_IT(&huart1, &RData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(5);
		/*BEGIN PID SELF-BALANCED*/
		while(MPU6050_Init(&hi2c1)==1);
		MPU6050_Read_All(&hi2c1, &MPU6050);
		AngleX = MPU6050.KalmanAngleX;// + GyroY*dt;
		AngleTarget = 0;
		if (RData == 0x32) AngleTarget =2.1;
		if (RData == 0x38) AngleTarget =-2.6;
		I = I + AngleX - AngleTarget;
		if (fabs(I)>=700) I = 700;
		Output = Kp*(AngleX - AngleTarget) + Kd*(AngleX - AngleBefore)/dt+Ki*I*dt;
		AngleBefore= AngleX;
		//Output = 1000;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, fabs(Output));
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, fabs(Output));
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, fabs(Output));
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, fabs(Output));
		/*END PID SELF-BALANCED*/
		
		/*BEGIN PID SPEED*/
		
/*
		Is = Is + Speed - SpeedTarget;
		OutputS = Kps*(Speed - SpeedTarget) + Kds*(Speed - SpeedBefore)/dt+Kis*Is*dt;
    Speed = ReadEncoder();
		SpeedBefore = Speed;
	*/
		/*END PID SPEED*/
		/*BEGIN CONTROL AND BALANCE*/
		switch (RData)
{
    case 0x32: {
     if ((Output > 0) && (AngleX <60)) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,  Output);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,  (Output));
			}
			if ((Output > 0) && (AngleX >=60)) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,  0);
			}
		  if ((Output < 0) && (AngleX > -60) ) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,fabs(Output));
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,fabs(Output));
			}
			if ((Output < 0) && (AngleX < -60) ) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
			}
      break;
			}
    case 0x34: {
			if ((Output > 0) && (AngleX <60)) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,  0.8*Output);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,    (Output));
			}
			if ((Output > 0) && (AngleX >=60)) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,  0);
			}
		  if ((Output < 0) && (AngleX > -60) ) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,fabs(Output));
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,   fabs(Output));
			}
			if ((Output < 0) && (AngleX < -60) ) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
			}
      break;
		}
		case 0x36: {
      if ((Output > 0) && (AngleX <60)) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,  Output);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0.8*Output);
			}
			if ((Output > 0) && (AngleX >=60)) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,  0);
			}
		  if ((Output < 0) && (AngleX > -60) ) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,fabs(Output));
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,   fabs(Output));
			}
			if ((Output < 0) && (AngleX < -60) ) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
			}
      break;
			}
		case 0x38: {
       if ((Output > 0) && (AngleX <60)) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,  Output);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,    (Output));
			}
			if ((Output > 0) && (AngleX >=60)) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,  0);
			}
			
		///////////////////////////////////////////////////////////////////////////	
			
			
			if ((Output < 0) && (AngleX > -60) ) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,fabs(Output));
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,fabs(Output));
			}
			if ((Output < 0) && (AngleX < -60) ) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
			}
      break;
			}
		////////////////////////////////////////////////////////////////////////
		case 0x35: {
      if ((Output > 0) && (AngleX <60)) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,  Output);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,    (Output));
			}
			if ((Output > 0) && (AngleX >=60)) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,  0);
			}
			
		///////////////////////////////////////////////////////////////////////////	
			
			
			if ((Output < 0) && (AngleX > -60) ) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,fabs(Output));
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2,   fabs(Output));
			}
			if ((Output < 0) && (AngleX < -60) ) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 0);
			}
      break;
			}
		//////////////////////////////////////////////////////////
    default: {
      if (Output > 0) {
				GoAhead();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Output);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, Output);
			}
			if (Output < 0) {
				GoBack();
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, fabs(Output));
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, fabs(Output));
			}
			}
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 40000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IN4_Pin */
  GPIO_InitStruct.Pin = IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IN4_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
		{
			HAL_UART_Receive_IT(&huart1, &RData, 1);
		}
		 
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
		
		
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
