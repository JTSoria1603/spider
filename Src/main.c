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
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN GPIO_PIN_13
#define TRIG_PORT GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
bool dato_disponible;
uint8_t byte;
uint8_t servo1 = 0;
uint8_t servo2 = 0;
uint8_t servo3 = 0;
uint8_t servo4 = 0;
uint8_t servo5 = 0;
uint8_t servo6 = 0;
uint8_t servo7 = 0;
uint8_t servo8 = 0;
uint8_t servo9 = 0;
uint8_t servo10 = 0;
uint8_t servo11 = 0;
uint8_t servo12 = 0;
uint8_t servo13 = 0;
uint8_t servo14 = 0;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0; //Is the first value captured?
uint8_t Distance = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint32_t Angle_Converter(uint32_t angle);
void HAL_Delay_us(uint16_t us);
void procesaComando(uint8_t byte);
void HCSR04_Read(void);

//-------------------------------------------MOVIMIENTOS DE CADA SERVO-----------------------------------------------------------------------------
//------------------------------------------------SERVO-MOTOR 1-----------------------------------------------------------------------------
//--------------------------------------------SERVOS 1 Y 2 CABEZA------------------------------------------------------------------------
//------------------------------------------- SERVOS IMPARES (INTERNOS)----------------------------------------------------------------
//--------------------------------------------SERVOS PARES (EXTERNOS)---------------------------------------------------------------------
void LeftServo1(void);
void RightServo1(void); 				//CHECK
void CentrarServo1(void);
//------------------------------------------------SERVO-MOTOR 2---------------------------------------------------------------
void UpServo2(void);
void DownServo2(void);
void CentrarServo2(void);
//------------------------------------------------SERVO-MOTOR 3---------------------------------------------------------------
void AvanceServo3(void);
void RetrocedeServo3(void);
void CentrarServo3(void); 
//------------------------------------------------SERVO-MOTOR 4---------------------------------------------------------------
void SubeServo4(void);
void BajaServo4(void);
void CentrarServo4(void);
//------------------------------------------------SERVO-MOTOR 5---------------------------------------------------------------
void AvanceServo5 (void);
void RetrocedeServo5(void);
void CentrarServo5(void);
//------------------------------------------------SERVO-MOTOR 6---------------------------------------------------------------
void SubeServo6(void);
void BajaServo6(void);
void CentrarServo6(void);
//------------------------------------------------SERVO-MOTOR 7---------------------------------------------------------------
void AvanceServo7(void);
void RetrocedeServo7(void);
void CentrarServo7(void);
//------------------------------------------------SERVO-MOTOR 8---------------------------------------------------------------
void SubeServo8(void);
void BajaServo8(void);
void CentrarServo8(void);
//------------------------------------------------SERVO-MOTOR 9---------------------------------------------------------------
void AvanceServo9(void);
void RetrocedeServo9(void);
void CentrarServo9(void);
//------------------------------------------------SERVO-MOTOR 10---------------------------------------------------------------
void SubeServo10(void);
void BajaServo10(void);
void CentrarServo10(void);
//------------------------------------------------SERVO-MOTOR 11---------------------------------------------------------------
void AvanceServo11(void);
void RetrocedeServo11(void);
void CentrarServo11(void);
//------------------------------------------------SERVO-MOTOR 12---------------------------------------------------------------
void SubeServo12(void);
void BajaServo12(void);
void CentrarServo12(void);
//------------------------------------------------SERVO-MOTOR 13---------------------------------------------------------------
void AvanceServo13 (void);
void RetrocedeServo13 (void);
void CentrarServo13(void);
//------------------------------------------------SERVO-MOTOR 14---------------------------------------------------------------
void SubeServo14(void);
void BajaServo14(void);
void CentrarServo14(void);
//-------------------------------------------MOVIMIENTOS DE CADA PATA-----------------------------------------------------------------------------
//----------------------------------------------------PATA 1-----------------------------------------------------------------------------
void AvanzaPata1(void);	
void CentrarPata1(void);	
void RetrocedePata1(void);	
//----------------------------------------------------PATA 2-----------------------------------------------------------------------------
void AvanzaPata2(void);	
void CentrarPata2(void);	
void RetrocedePata2(void);	
//----------------------------------------------------PATA 3-----------------------------------------------------------------------------
void AvanzaPata3(void);
void CentrarPata3(void);
void RetrocedePata3(void);
//----------------------------------------------------PATA 4-----------------------------------------------------------------------------
void AvanzaPata4(void);
void CentrarPata4(void);
void RetrocedePata4(void);
//----------------------------------------------------PATA 5-----------------------------------------------------------------------------
void AvanzaPata5(void);
void CentrarPata5(void);
void RetrocedePata5(void);
//----------------------------------------------------PATA 6-----------------------------------------------------------------------------
void AvanzaPata6(void);
void CentrarPata6(void);
void RetrocedePata6(void);
//-------------------------------------------MOVIMIENTOS DEL HEXAPODO-----------------------------------------------------------------------------
void Saludar(void);
void Centrar(void);
void Avanzar(void);
void Retroceder(void);
void Turn_Left(void);
void Turn_Right(void);
void Defense(void);
void Attack(void);
void Explore(void);
//--------------------------------------------------------------------------------------------------------------------------------------------------

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_IT(&huart3, (uint8_t*) &byte,1);
	
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 1
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 3
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 4

HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 1
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 2
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 3
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 4

HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 1
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 2
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 3
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 4

HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 1
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 2
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); //Inicializamos el Generador PWM del TIMER 1 para el CHANNEL 3

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
		HCSR04_Read();
		if(Distance < 6)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);		//LED VERDE
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET); //LED ROJO
		}
		if(Distance >= 6)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET); //LED ROJO
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);	//LED VERDE
		}
		HAL_Delay(200); //Before initializate next measurenment, wait at least 60ms
		HAL_TIM_Base_Stop(&htim1);
		HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
		
		if(dato_disponible) //Preguntamos si se envi� alg�n caracter por Bluetooth
		{
			dato_disponible = false;
			procesaComando(byte); //Leemos el caracter recibido
			}
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB13 PB14 PB15 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t Angle_Converter(uint32_t angle)
{
	switch((int)angle)
	{
		case 1:
			angle=499;
			break;
		case 2:
			angle=999;
			break;
		case 3:
			angle=1499;
		case 4:
			angle=1999;
			break;
		default:
			break;
	}
}

void HAL_Delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0); //Set the counter value to 0
	while(__HAL_TIM_GET_COUNTER(&htim1) < us); //Wait for the counter to reach the us input in the parameter
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //Utilizamos esta funci�n como interrupci�n y se ejecuta cada vez que recibimos un valor en el puerto serial
{
	dato_disponible = true;
	HAL_UART_Receive_IT(&huart3,(uint8_t*) &byte,1);
}

void procesaComando(uint8_t byte) //Procesamos el caracter recibido
{
	switch ((char) byte) //Debo configurar el switch
	{
		case 'A': //Ataque
			Attack();	
			break;
		case 'B': //Bloqueo
			Defense();
			break;
		case 'C': //Centrar
			Centrar();
			break;
		case 'H': //Saludar
			Saludar();
			break;
		case 'D': //Rotar a Derecha
			Turn_Right();
			break;
		case 'I': //Rotar a Izquierda
			Turn_Left();	
			break;
		case 'E': //Explorar
			Explore();	
			break;
		case 'F': //Foward o Avanzar
			Avanzar();
			break;
		case 'R': //Reverse o Retroceder
			Retroceder();
			break;
		case 'G': //Servo1
			if(servo1 == 0)
			{
				servo1 = 1;
				LeftServo1();
			}
			else{
			if(servo1 == 1)
			{
				servo1 = 2;
				CentrarServo1();
			}
			else
			{
				servo1 = 0;
				RightServo1();
			}
			}
			break;
		case 'J': //Servo2
			if(servo2 == 0)
			{
				servo2 = 1;
				UpServo2();
			}
			else{
			if(servo2 == 1)
			{
				servo2 = 2;
				CentrarServo2();
			}
			else
			{
				servo2 = 0;
				DownServo2();
			}
		}
			break;
		case 'K': //Servo3
			if(servo3 == 0)
			{
				servo3 = 1;
				AvanceServo3();
			}
			else{
			if(servo3 == 1)
			{
				servo3 = 2;
				CentrarServo3();
			}
			else
			{
				servo3 = 0;
				RetrocedeServo3();
			}
		}
			break;
		case 'L': //Servo4
			if(servo4 == 0)
			{
				servo4 = 1;
				SubeServo4();
			}
			else{
			if(servo4 == 1)
			{
				servo4 = 2;
				CentrarServo4();
			}
			else
			{
				servo4 = 0;
				BajaServo4();
			}
		}
			break;
		case 'M': //Servo5
			if(servo5 == 0)
			{
				servo5 = 1;
				AvanceServo5();
			}
			else{
			if(servo5 == 1)
			{
				servo5 = 2;
				CentrarServo5();
			}
			else
			{
				servo5 = 0;
				RetrocedeServo5();
			}
		}
			break;
		case 'N': //Servo6
			if(servo6 == 0)
			{
				servo6 = 1;
				SubeServo6();
			}
			else{
			if(servo6 == 1)
			{
				servo6 = 2;
				CentrarServo6();
			}
			else
			{
				servo6 = 0;
				BajaServo6();
			}
		}
			break;
		case 'O': //Servo7
			if(servo7 == 0)
			{
				servo7 = 1;
				AvanceServo7();
			}
			else{
			if(servo7 == 1)
			{
				servo7 = 2;
				CentrarServo7();
			}
			else
			{
				servo7 = 0;
				RetrocedeServo7();
			}
		}
			break;
		case 'P': //Servo8
			if(servo8 == 0)
			{
				servo8 = 1;
				SubeServo8();
			}
			else{
			if(servo8 == 1)
			{
				servo8 = 2;
				CentrarServo8();
			}
			else
			{
				servo8 = 0;
				BajaServo8();
			}
		}
			break;
		case 'Q': //Servo9
			if(servo9 == 0)
			{
				servo9 = 1;
				AvanceServo9();
			}
			else{
			if(servo9 == 1)
			{
				servo9 = 2;
				CentrarServo9();
			}
			else
			{
				servo9 = 0;
				RetrocedeServo9();
			}
		}
			break;
		case 'T': //Servo10
			if(servo10 == 0)
			{
				servo10 = 1;
				SubeServo10();
			}
			else{
			if(servo10 == 1)
			{
				servo10 = 2;
				CentrarServo10();
			}
			else
			{
				servo10 = 0;
				BajaServo10();
			}
		}
			break;
		case 'U': //Servo11
			if(servo11 == 0)
			{
				servo11 = 1;
				AvanceServo11();
			}
			else{
			if(servo11 == 1)
			{
				servo11 = 2;
				CentrarServo11();
			}
			else
			{
				servo11 = 0;
				RetrocedeServo11();
			}
		}
			break;
		case 'V': //Servo12
			if(servo12 == 0)
			{
				servo12 = 1;
				SubeServo12();
			}
			else{
			if(servo12 == 1)
			{
				servo12 = 2;
				CentrarServo12();
			}
			else
			{
				servo12 = 0;
				BajaServo12();
			}
		}
			break;
		case 'W': //Servo13
			if(servo13 == 0)
			{
				servo13 = 1;
				AvanceServo13();
			}
			else{
			if(servo13 == 1)
			{
				servo13 = 2;
				CentrarServo13();
			}
			else
			{
				servo13 = 0;
				RetrocedeServo13();
			}
		}
			break;
		case 'X': //Servo 14
			if(servo14 == 0)
			{
				servo14 = 1;
				SubeServo14();
			}
			else{
			if(servo14 == 1)
			{
				servo14 = 2;
				CentrarServo14();
			}
			else
			{
				servo14 = 0;
				BajaServo14();
			}
		}
			break;
		default:
			break;
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) //If the first value is not captured
	{
		if(Is_First_Captured == 0) //If the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //Read the first value
			Is_First_Captured = 1; //Set the first captured as true
			//Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if(Is_First_Captured == 1) //If the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); //Read the first value
			__HAL_TIM_SET_COUNTER(htim,0); //Reset the counter
			
			if(IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2 - IC_Val1;
			}
			else if(IC_Val1 > IC_Val2)
			{
				Difference = (19999-IC_Val1) + IC_Val2;
			}
			Distance = Difference * 0.034/2;
			Is_First_Captured = 0; //Set it back to false
			
			//Set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC2);
		}
	}
}
void HCSR04_Read(void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); //Pull the TRIG pin HIGH
	HAL_Delay_us(10); //Wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); //Pull the TRIG pin LOW
	
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
}
//-------------------------------------------MOVIMIENTOS DEL HEXAPODO-----------------------------------------------------------------------------
void Saludar(void)
{
	Centrar();
	SubeServo4();
	AvanceServo3();
	RetrocedeServo3();
	AvanceServo3();
	RetrocedeServo3();
	CentrarServo3();
	BajaServo4();
}
void Centrar(void)
{
	CentrarPata1();
	CentrarPata2();
	CentrarPata3();
	CentrarPata4();
	CentrarPata5();
	CentrarPata6();
}
void Avanzar(void)
{
	AvanzaPata1();
	AvanzaPata3();
	AvanzaPata5();
	CentrarServo3();
	CentrarServo7();
	CentrarServo11();
	AvanzaPata2();
	AvanzaPata4();
	AvanzaPata6();
	CentrarServo5();
	CentrarServo9();
	CentrarServo13();
}
void Retroceder(void)
{
	RetrocedePata1();
	RetrocedePata3();
	RetrocedePata5();
	CentrarServo3();
	CentrarServo7();
	CentrarServo11();
	RetrocedePata2();
	RetrocedePata4();
	RetrocedePata6();
	CentrarServo5();
	CentrarServo9();
	CentrarServo13();
}
void Turn_Left(void)
{
	AvanzaPata6();
	AvanzaPata4();
	RetrocedePata2();
	RetrocedePata1();
	RetrocedePata3();
	AvanzaPata5();
}
void Turn_Right(void)
{
	RetrocedePata6();
	RetrocedePata4();
	AvanzaPata2();
	AvanzaPata1();
	AvanzaPata3();
	RetrocedePata5();
}
void Attack(void)
{
	Centrar();
	AvanceServo3();
	AvanceServo13();
	SubeServo4();
	SubeServo14();
	BajaServo4();
	BajaServo14();
}
void Defense(void)
{
	Centrar();
	CentrarServo4();
	CentrarServo10();
	CentrarServo12();
	CentrarServo6();
	CentrarServo8();
	CentrarServo14();
}
void Explore(void)
{
			HCSR04_Read();
		if(Distance < 6)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
		}
		if(Distance >= 6)
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
		}
		HAL_Delay(200); //Before initializate next measurenment, wait at least 60ms
}
//-------------------------------------------MOVIMIENTOS DE CADA PATA-----------------------------------------------------------------------------
//----------------------------------------------------PATA 1-----------------------------------------------------------------------------
void AvanzaPata1(void)
{
	SubeServo4();
	AvanceServo3();
	BajaServo4();
}
void CentrarPata1(void)
{
	SubeServo4();
	CentrarServo3();
	BajaServo4();
}
void RetrocedePata1(void)
{
	SubeServo4();
	RetrocedeServo3();
	BajaServo4();
}
//----------------------------------------------------PATA 2-----------------------------------------------------------------------------
void AvanzaPata2(void)
{
	SubeServo6();
	AvanceServo5();
	BajaServo6();
}
void CentrarPata2(void)
{
	SubeServo6();
	CentrarServo5();
	BajaServo6();
}
void RetrocedePata2(void)
{
	SubeServo6();
	RetrocedeServo5();
	BajaServo6();
}
//----------------------------------------------------PATA 3-----------------------------------------------------------------------------
void AvanzaPata3(void)
{
	SubeServo8();
	AvanceServo7();
	BajaServo8();
}
void CentrarPata3(void)
{
	SubeServo8();
	CentrarServo7();
	BajaServo8();
}
void RetrocedePata3(void)
{
	SubeServo8();
	RetrocedeServo7();
	BajaServo8();
}
//----------------------------------------------------PATA 4-----------------------------------------------------------------------------
void AvanzaPata4(void)
{
	SubeServo10();
	AvanceServo9();
	BajaServo10();
}
void CentrarPata4(void)
{
	SubeServo10();
	CentrarServo9();
	BajaServo10();
}
void RetrocedePata4(void)
{
	SubeServo10();
	RetrocedeServo9();
	BajaServo10();
}
//----------------------------------------------------PATA 5-----------------------------------------------------------------------------
void AvanzaPata5(void)
{
	SubeServo12();
	AvanceServo11();
	BajaServo12();
}
void CentrarPata5(void)
{
	SubeServo12();
	CentrarServo11();
	BajaServo12();
}
void RetrocedePata5(void)
{
	SubeServo12();
	RetrocedeServo11();
	BajaServo12();
}
//----------------------------------------------------PATA 6-----------------------------------------------------------------------------
void AvanzaPata6(void)
{
	SubeServo14();
	AvanceServo13();
	BajaServo14();
}
void CentrarPata6(void)
{
	SubeServo14();
	CentrarServo13();
	BajaServo14();
}
void RetrocedePata6(void)
{
	SubeServo14();
	RetrocedeServo13();
	BajaServo14();
}
//----------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------MOVIMIENTOS DE CADA SERVO-----------------------------------------------------------------------------
//------------------------------------------------SERVO-MOTOR 1-----------------------------------------------------------------------------
//CABLITRATE

void LeftServo1(void)
{
			TIM1->CCR1 = 499;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(500);	
}
void CentrarServo1(void)
{
			TIM1->CCR1 = 999;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(500);
}
void RightServo1(void) //Revisar porque no est� respondiendo
{
			TIM1->CCR1 = 1499;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_Delay(500);		
}

//------------------------------------------------SERVO-MOTOR 2-----------------------------------------------------------------------------
void CentrarServo2(void)
{ 
			TIM1->CCR3 = 1499;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_Delay(500);		
}
void UpServo2(void)
{
			TIM1->CCR3 = 1999;
			HAL_TIM_PWM_Start
	(&htim1, TIM_CHANNEL_3);
			HAL_Delay(500);		
}
void DownServo2(void)
{
			TIM1->CCR3 = 999;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
			HAL_Delay(500);	
}
//------------------------------------------------SERVO-MOTOR 3-----------------------------------------------------------------------------
void AvanceServo3(void)
{
			TIM1->CCR4 = 1999;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

void CentrarServo3(void)
{
			TIM1->CCR4 = 1499;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

void RetrocedeServo3(void)
{
			TIM1->CCR4 = 499;
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 4-----------------------------------------------------------------------------
void CentrarServo4(void)
{
			TIM2->CCR1 = 999;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_Delay(500);	
}
void SubeServo4(void)
{
			TIM2->CCR1 = 1999;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_Delay(500);	
}

void BajaServo4(void)
{
			TIM2->CCR1 = 499;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_Delay(500);	
}
//------------------------------------------------SERVO-MOTOR 5-----------------------------------------------------------------------------
void RetrocedeServo5(void)
{
			TIM2->CCR2 = 999;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			HAL_Delay(500);		
}

void CentrarServo5(void)
{
			TIM2->CCR2 = 1499;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2	);
			HAL_Delay(500);	
}

void AvanceServo5(void)
{
			TIM2->CCR2 = 1999;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 6-----------------------------------------------------------------------------
void SubeServo6(void)
{
			TIM2->CCR3 = 1999;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

void CentrarServo6(void)
{
			TIM2->CCR3 = 1499;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

void BajaServo6(void)
{
			TIM2->CCR3 = 499;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
			HAL_Delay(500);		
}

//------------------------------------------------SERVO-MOTOR 7-----------------------------------------------------------------------------
void AvanceServo7(void)
{
			TIM2->CCR4 = 1499;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

void RetrocedeServo7(void)
{
			TIM2->CCR4 = 499;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

void CentrarServo7(void)
{
			TIM2->CCR4 = 999;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_Delay(500);		
}

//------------------------------------------------SERVO-MOTOR 8-----------------------------------------------------------------------------
void SubeServo8(void)
{
			TIM3->CCR1 = 1999;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_Delay(500);		
}

void BajaServo8(void)
{
			TIM3->CCR1 = 499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_Delay(500);	
}

void CentrarServo8(void)
{
			TIM3->CCR1 = 1499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 9-----------------------------------------------------------------------------
void RetrocedeServo9(void)
{
			TIM3->CCR2 = 1499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

void CentrarServo9(void)
{
			TIM3->CCR2 = 999;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

void AvanceServo9(void)
{
			TIM3->CCR2 = 499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 10-----------------------------------------------------------------------------
void SubeServo10(void)
{
			TIM3->CCR3 = 1999;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

void BajaServo10(void)
{
			TIM3->CCR3 = 499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

void CentrarServo10(void)
{
			TIM3->CCR3 = 1499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 11-----------------------------------------------------------------------------
void AvanceServo11(void)
{
			TIM3->CCR4 = 499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

void CentrarServo11(void)
{
			TIM3->CCR4 = 999;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
			HAL_Delay(500);	
}

void RetrocedeServo11(void)
{
			TIM3->CCR4 = 1499;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
			HAL_Delay(500);	
}


//------------------------------------------------SERVO-MOTOR 12-----------------------------------------------------------------------------
void SubeServo12(void)
{
			TIM4->CCR1 = 1999;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			HAL_Delay(500);	
}

void CentrarServo12(void)
{
			TIM4->CCR1 = 1499;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			HAL_Delay(500);	
}

void BajaServo12(void)
{
			TIM4->CCR1 = 499;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 13-----------------------------------------------------------------------------
void AvanceServo13(void)
{
			TIM4->CCR2 = 499;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

void RetrocedeServo13(void)
{
			TIM4->CCR2 = 1499;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

void CentrarServo13(void)
{
			TIM4->CCR2 = 999;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			HAL_Delay(500);	
}

//------------------------------------------------SERVO-MOTOR 14-----------------------------------------------------------------------------
void SubeServo14(void)
{
			TIM4->CCR3 = 1999;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

void CentrarServo14(void)
{
			TIM4->CCR3 = 1499;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_Delay(500);	
}

void BajaServo14(void)
{
			TIM4->CCR3 = 499;
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			HAL_Delay(500);	
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
