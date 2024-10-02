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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DCMotor1 */
osThreadId_t DCMotor1Handle;
const osThreadAttr_t DCMotor1_attributes = {
  .name = "DCMotor1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Encoder */
osThreadId_t EncoderHandle;
const osThreadAttr_t Encoder_attributes = {
  .name = "Encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_Thread */
osThreadId_t UART_ThreadHandle;
const osThreadAttr_t UART_Thread_attributes = {
  .name = "UART_Thread",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// Define the attributes for ProcessUartTask
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void *argument);
void motors(char command, int distance);
void encoder(void *argument);
void Uart_Function(void *argument);
void forward(int distance);
void backward(int distance);
void frontRight(int distance);
void frontLeft(int distance);
void backRight(int distance);
void backLeft(int distance);
uint32_t countTargetTicks(int distance);

/* USER CODE BEGIN PFP */
float Kp = 2.0, Ki = 0.1, Kd = 0.05;  // PID constants
float errorLeft, errorRight;
float integralLeft = 0, integralRight = 0;
float derivativeLeft, derivativeRight;
float previousErrorLeft = 0, previousErrorRight = 0;
int targetSpeedLeft = 2000;  // Target speed for left motor
int targetSpeedRight = 2000; // Target speed for right motor
int pwmLeft = 0, pwmRight = 0;  // PWM values for motors
uint32_t previousTime = 0;

#define WHEEL_DIAMETER_CM 6
#define PI 3.14159
#define TICKS_PER_REV 390

// Variables to store encoder counts
volatile uint32_t left_encoder_count = 0;
volatile uint32_t right_encoder_count = 0;
volatile int startFlag = 0;
volatile int uartReceived = 0;

uint32_t target_counts;
void UltraSonic(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxBuffer[5];
uint8_t rpiBuffer[5];
uint8_t instructionBuffer[1000];
uint32_t instructionIndex = 0;
uint8_t aRxBuffer[5];
double detectedDistance;
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  //HAL_UART_Receive_IT(&huart3,aRxBuffer,5);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DCMotor1 */
  //DCMotor1Handle = osThreadNew(motors, NULL, &DCMotor1_attributes);

  /* creation of Encoder */
  EncoderHandle = osThreadNew(encoder, NULL, &Encoder_attributes);

  /* creation of UART_Thread */
  UART_ThreadHandle = osThreadNew(Uart_Function, NULL, &UART_Thread_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* USER CODE BEGIN 3 */
	  /* USER CODE END 3 */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ICM_SCL_Pin ICM_SDA_Pin */
  GPIO_InitStruct.Pin = ICM_SCL_Pin|ICM_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Mark that data has been received
    //uartReceived = 1;
    // Optionally, for debugging, show it on the OLED in the main thread
    ////OLED_Clear();
    //OLED_ShowString(10, 10, "testing in uart");
    //HAL_UART_Transmit(&huart3, rxBuffer, 1, 1000);
    ////OLED_ShowString(10, 20, rxBuffer);
    ////OLED_Refresh_Gram();
    if(rxBuffer[0]!='#'){
    	if(instructionIndex<999){
    		instructionBuffer[instructionIndex] = rxBuffer[0];
    		instructionBuffer[++instructionIndex] = rxBuffer[1];
    		instructionBuffer[++instructionIndex] = rxBuffer[2];
    		instructionBuffer[++instructionIndex] = rxBuffer[3];
    		instructionBuffer[++instructionIndex] = rxBuffer[4];
    		instructionIndex++; //so that index start from 0
    		HAL_UART_Receive_IT(&huart3, rxBuffer, 5);
    	}
    	else{
    		OLED_Clear();
    		OLED_ShowString(10,30,"bufferOverflow");
    	}
    }
    else{
    	startFlag = 1;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	static int tc1, tc2, first=0, echo = 0;
	char buf[15];
	if(htim==&htim4){

		if (first == 0){
			tc1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			first=1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (first == 1){
			tc2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim, 0);

			if(tc2 >= tc1){
				echo = tc2 - tc1;
			} else{
				echo = (0xffff - tc1) + tc2;
			}

			sprintf(buf, "Echo = %5dus", echo);
			OLED_ShowString(10, 40, &buf[0]);
			OLED_Refresh_Gram();
			detectedDistance = echo * (0.0343/2);
			//Can use this global variable to stop the motors
			// For example
			// if(detectedDistance<=30){
			//  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
    	    //	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
    	    //	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
    	    //	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
    	    //	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
    	    //	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
			//} the else just do what ever other movement type bah
			sprintf(buf, "Dist = %5.1fcm", detectedDistance);
			OLED_ShowString(10, 50, &buf[0]);
			OLED_Refresh_Gram();

			first=0;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
		}
	}
}

void UltraSonic(void){

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);

	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while(__HAL_TIM_GET_COUNTER(&htim6)<=10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}
uint32_t countTargetTicks(int distance){

	uint32_t rev,targetTicks;
	float dist, circumference;
	dist = distance;
	circumference = PI * WHEEL_DIAMETER_CM ;
	rev = dist/circumference;
	targetTicks = rev * TICKS_PER_REV * 4;

	return targetTicks;
}

void forward(int distance){

//	char buff[9];
//	OLED_Clear();
//	sprintf(buff,"forward %d",distance);
//	OLED_ShowString(10,10,buff);

  	uint32_t pwmVal = 2000;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);;

  			//(uint32_t)countTargetTicks(distance);
 	char tbuff[9];
 	sprintf(tbuff,"Ticks = %d",targetTicks);
 	OLED_Clear();
 	OLED_ShowString(10,0,tbuff);
 	OLED_Refresh_Gram();
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 155;

  	for(;;){
  	  //Move Forward
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmVal);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmVal);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (encoderCount>=targetTicks) {
              // Stop the motor
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              return;
          }
  	}
}
void backward(int distance){
	char buff[9];
	OLED_Clear();
	sprintf(buff,"backward %d",distance);
	OLED_ShowString(10,10,buff);
  	uint32_t pwmVal = 2000;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

  			//(uint32_t)countTargetTicks(distance);
 	char tbuff[9];
 	sprintf(tbuff,"Ticks = %d",targetTicks);
 	OLED_Clear();
 	OLED_ShowString(10,0,tbuff);
 	OLED_Refresh_Gram();
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 155;
  	for(;;){
  	  //Move Backwards
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmVal);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmVal);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (encoderCount>=targetTicks) {
              // Stop the motor
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              return;
          }
  	}

}
void frontRight(int distance){
	distance = 75; //set as fixed value
  	uint32_t pwmVal = 2000;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

  			//(uint32_t)countTargetTicks(distance);
 	char tbuff[9];
 	sprintf(tbuff,"Ticks = %d",targetTicks);
 	OLED_Clear();
 	OLED_ShowString(10,0,tbuff);
 	OLED_Refresh_Gram();
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 230;

  	for(;;){
  	  //Move Forward
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmVal);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmVal);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (encoderCount>=targetTicks) {
              // Stop the motor
        	  htim1.Instance -> CCR4 = 155;
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              return;
          }
  	}


}
void frontLeft(int distance) {
	distance = 75; //set as fixed value
  	uint32_t pwmVal = 2000;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

  			//(uint32_t)countTargetTicks(distance);
 	char tbuff[9];
 	sprintf(tbuff,"Ticks = %d",targetTicks);
 	OLED_Clear();
 	OLED_ShowString(10,0,tbuff);
 	OLED_Refresh_Gram();
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 90;

  	for(;;){
  	  //Move Forward
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmVal);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmVal);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (encoderCount>=targetTicks) {
              // Stop the motor
        	  htim1.Instance -> CCR4 = 155;
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              return;
          }
  	}

}
void backRight(int distance){
	distance = 75; //set as fixed value
  	uint32_t pwmVal = 2000;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

  			//(uint32_t)countTargetTicks(distance);
 	char tbuff[9];
 	sprintf(tbuff,"Ticks = %d",targetTicks);
 	OLED_Clear();
 	OLED_ShowString(10,0,tbuff);
 	OLED_Refresh_Gram();
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 90;

  	for(;;){
  	  //Move Backwards
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmVal);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmVal);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (encoderCount>=targetTicks) {
        	  htim1.Instance -> CCR4 = 155;
              // Stop the motor
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              return;
          }
  	}
}
void backLeft(int distance){
	distance = 75; //set as fixed value
  	uint32_t pwmVal = 2000;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

  			//(uint32_t)countTargetTicks(distance);
 	char tbuff[9];
 	sprintf(tbuff,"Ticks = %d",targetTicks);
 	OLED_Clear();
 	OLED_ShowString(10,0,tbuff);
 	OLED_Refresh_Gram();
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 230;

  	for(;;){
  	  //Move Backwards
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmVal);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmVal);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (encoderCount>=targetTicks) {
        	  htim1.Instance -> CCR4 = 155;
              // Stop the motor
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              return;
          }
  	}
}
/*
void System_Init() {
    // Initialize GPIO, Timers, PWM, and Encoder inputs here
    // Start encoder timers in interrupt mode
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // Example for left wheel
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // Example for right wheel

    // Initialize motor drivers (PWM setup)
    // e.g., HAL_TIM_PWM_Start for motor PWM control
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {  // Left wheel encoder
        left_encoder_count++;
    } else if (htim->Instance == TIM2) {  // Right wheel encoder
        right_encoder_count++;
    }
}

void Encoder_Init() {
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // Start the left motor encoder
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // Start the right motor encoder
}
*/

/*
void HAL_UART_RxCpltCallBack(UART_HandleTypeDef *huart)
{
	UNUSED(huart);

	HAL_UART_Transmit(&huart3,(uint8_t*)aRxBuffer,10,0xFFFF);
}
*/
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //uint8_t ch = 'A';
  for(;;)
  {
	//HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,0xFFFF);
	//if (ch < 'Z'){
		//ch++;
	//}
	//else{
		//ch = 'A';
	//}
	  osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motors */
/**
* @brief Function implementing the DCMotor1 thread.
* @param argument: Not used
* @retval None
*/


void motors(char command, int distance)
{
    //USER CODE BEGIN motors
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    uint32_t encoderCount = 0;
    uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t currentEncoderCount = 0;
    float wheelCircumference = WHEEL_DIAMETER_CM * PI;

        // Step 2: Calculate the number of wheel revolutions needed to cover the target distance
      // Adjust according to your distance calculation
    uint32_t previousTime = HAL_GetTick();  // For calculating time interval (dt)

    // PID Constants (adjust as needed)
    float Kp = 2.0;
    float Ki = 0.1;
    float Kd = 0.5;

    // Initialize PID control variables
    float errorLeft = 0, errorRight = 0;
    float integralLeft = 0, integralRight = 0;
    float derivativeLeft = 0, derivativeRight = 0;
    float previousErrorLeft = 0, previousErrorRight = 0;
    int pwmLeft = 0, pwmRight = 0;

    OLED_ShowString(10, 50, "Test in motor");

    for (;;)
    {
        switch (command) {
        case 'F': // Forward
            OLED_ShowString(10, 10, "F Received");
            int numRevolutions = distance / (int)wheelCircumference;
            uint32_t targetTicks = (uint32_t)(numRevolutions * TICKS_PER_REV);
            htim1.Instance->CCR4 = 155;
            // Move Forward (Set motor direction for both motors)
            HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);  // Left motor forward
            HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);  // Right motor forward

            // Calculate encoder counts for distance
            currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
            encoderCount = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ?
                            (startEncoderCount - currentEncoderCount) :
                            (currentEncoderCount - startEncoderCount);

            // PID control calculations
            uint32_t currentTime = HAL_GetTick();
            float dt = (currentTime - previousTime) / 1000.0f;  // Calculate time difference in seconds
            previousTime = currentTime;

            errorLeft = targetTicks - encoderCount;  // Assume symmetric errors for simplicity
            errorRight = errorLeft;

            // Proportional term
            float proportionalLeft = Kp * errorLeft;
            float proportionalRight = Kp * errorRight;

            // Integral term (accumulate error over time)
            integralLeft += errorLeft * dt;
            integralRight += errorRight * dt;
            float integralTermLeft = Ki * integralLeft;
            float integralTermRight = Ki * integralRight;

            // Derivative term (rate of change of error)
            derivativeLeft = (errorLeft - previousErrorLeft) / dt;
            derivativeRight = (errorRight - previousErrorRight) / dt;
            float derivativeTermLeft = Kd * derivativeLeft;
            float derivativeTermRight = Kd * derivativeRight;

            // Update the previous errors for the next iteration
            previousErrorLeft = errorLeft;
            previousErrorRight = errorRight;

            // Calculate the final PWM adjustment using the PID formula
            pwmLeft = proportionalLeft + integralTermLeft + derivativeTermLeft;
            pwmRight = proportionalRight + integralTermRight + derivativeTermRight;

            // Limit the PWM values to a valid range (0 to 2048 in this case)
            pwmLeft = (pwmLeft > 1900) ? 1900 : (pwmLeft < 0 ? 0 : pwmLeft);
            pwmRight = (pwmRight > 2048) ? 2048 : (pwmRight < 0 ? 0 : pwmRight);

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmLeft);  // Adjust left motor PWM
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmRight); // Adjust right motor PWM

            // Check if the robot has reached the target distance
            if (encoderCount >= targetTicks) {
                // Stop the motor
                HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
                HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
                return;  // Exit loop if target reached
            }
            htim1.Instance -> CCR4 = 155;
            break;

        case 'L': // Turn left
            htim1.Instance->CCR4 = 90;
            numRevolutions = 50 / (int)wheelCircumference;
            targetTicks = (uint32_t)(numRevolutions * TICKS_PER_REV);
            OLED_ShowString(10, 10, "L Received - Turn Left");
            // Move Forward (Set motor direction for both motors)
            HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);  // Left motor forward
            HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);  // Right motor backward
            HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
            currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
			encoderCount = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ?
							(startEncoderCount - currentEncoderCount) :
							(currentEncoderCount - startEncoderCount);

			// PID control calculations
			currentTime = HAL_GetTick();
			dt = (currentTime - previousTime) / 1000.0f;  // Calculate time difference in seconds
			previousTime = currentTime;

			errorLeft = targetTicks - encoderCount;  // Assume symmetric errors for simplicity
			errorRight = errorLeft;

			// Proportional term
			proportionalLeft = Kp * errorLeft;
			proportionalRight = Kp * errorRight;

			// Integral term (accumulate error over time)
			integralLeft += errorLeft * dt;
			integralRight += errorRight * dt;
			integralTermLeft = Ki * integralLeft;
			integralTermRight = Ki * integralRight;

			// Derivative term (rate of change of error)
			derivativeLeft = (errorLeft - previousErrorLeft) / dt;
			derivativeRight = (errorRight - previousErrorRight) / dt;
			derivativeTermLeft = Kd * derivativeLeft;
			 derivativeTermRight = Kd * derivativeRight;

			// Update the previous errors for the next iteration
			previousErrorLeft = errorLeft;
			previousErrorRight = errorRight;

			// Calculate the final PWM adjustment using the PID formula
			pwmLeft = proportionalLeft + integralTermLeft + derivativeTermLeft;
			pwmRight = proportionalRight + integralTermRight + derivativeTermRight;

			// Limit the PWM values to a valid range (0 to 2048 in this case)
			pwmLeft = (pwmLeft > 1900) ? 1900 : (pwmLeft < 0 ? 0 : pwmLeft);
			pwmRight = (pwmRight > 2048) ? 2048 : (pwmRight < 0 ? 0 : pwmRight);

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmLeft);  // Adjust left motor PWM
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmRight); // Adjust right motor PWM

			// Check if the robot has reached the target distance
			if (encoderCount >= targetTicks) {
				// Stop the motor
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
				return;  // Exit loop if target reached
			}
			htim1.Instance -> CCR4 = 155;
			break;

        case 'R': // Turn right
            htim1.Instance->CCR4 = 230;
            numRevolutions = 50 / (int)wheelCircumference;
            targetTicks = (uint32_t)(numRevolutions * TICKS_PER_REV);
            OLED_ShowString(10, 10, "R Received - Turn Right");
            // Move Forward (Set motor direction for both motors)
            HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);   // Left motor backward
            HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);  // Right motor forward
            currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
			encoderCount = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ?
							(startEncoderCount - currentEncoderCount) :
							(currentEncoderCount - startEncoderCount);

			// PID control calculations
			currentTime = HAL_GetTick();
			dt = (currentTime - previousTime) / 1000.0f;  // Calculate time difference in seconds
			previousTime = currentTime;

			errorLeft = targetTicks - encoderCount;  // Assume symmetric errors for simplicity
			errorRight = errorLeft;

			// Proportional term
			proportionalLeft = Kp * errorLeft;
			proportionalRight = Kp * errorRight;

			// Integral term (accumulate error over time)
			integralLeft += errorLeft * dt;
			integralRight += errorRight * dt;
			integralTermLeft = Ki * integralLeft;
			integralTermRight = Ki * integralRight;

			// Derivative term (rate of change of error)
			derivativeLeft = (errorLeft - previousErrorLeft) / dt;
			derivativeRight = (errorRight - previousErrorRight) / dt;
			derivativeTermLeft = Kd * derivativeLeft;
			derivativeTermRight = Kd * derivativeRight;

			// Update the previous errors for the next iteration
			previousErrorLeft = errorLeft;
			previousErrorRight = errorRight;

			// Calculate the final PWM adjustment using the PID formula
			pwmLeft = proportionalLeft + integralTermLeft + derivativeTermLeft;
			pwmRight = proportionalRight + integralTermRight + derivativeTermRight;

			// Limit the PWM values to a valid range (0 to 2048 in this case)
			pwmLeft = (pwmLeft > 1900) ? 1900 : (pwmLeft < 0 ? 0 : pwmLeft);
			pwmRight = (pwmRight > 2048) ? 2048 : (pwmRight < 0 ? 0 : pwmRight);

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmLeft);  // Adjust left motor PWM
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmRight); // Adjust right motor PWM

			// Check if the robot has reached the target distance
			if (encoderCount >= targetTicks) {
				// Stop the motor
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
				return;  // Exit loop if target reached
			}
			htim1.Instance -> CCR4 = 155;
			break;

        case 'B': // Move backward
            htim1.Instance->CCR4 = 155;
            numRevolutions = distance / (int)wheelCircumference;
            targetTicks = (uint32_t)(numRevolutions * TICKS_PER_REV);
            OLED_ShowString(10, 10, "B Received - Move Backward");
            HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);   // Left motor backward
            HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);   // Right motor backward
            HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
            currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
			encoderCount = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ?
							(startEncoderCount - currentEncoderCount) :
							(currentEncoderCount - startEncoderCount);

			// PID control calculations
			currentTime = HAL_GetTick();
			dt = (currentTime - previousTime) / 1000.0f;  // Calculate time difference in seconds
			previousTime = currentTime;

			errorLeft = targetTicks - encoderCount;  // Assume symmetric errors for simplicity
			errorRight = errorLeft;

			// Proportional term
			proportionalLeft = Kp * errorLeft;
			proportionalRight = Kp * errorRight;

			// Integral term (accumulate error over time)
			integralLeft += errorLeft * dt;
			integralRight += errorRight * dt;
			integralTermLeft = Ki * integralLeft;
			integralTermRight = Ki * integralRight;

			// Derivative term (rate of change of error)
			derivativeLeft = (errorLeft - previousErrorLeft) / dt;
			derivativeRight = (errorRight - previousErrorRight) / dt;
			derivativeTermLeft = Kd * derivativeLeft;
			derivativeTermRight = Kd * derivativeRight;

			// Update the previous errors for the next iteration
			previousErrorLeft = errorLeft;
			previousErrorRight = errorRight;

			// Calculate the final PWM adjustment using the PID formula
			pwmLeft = proportionalLeft + integralTermLeft + derivativeTermLeft;
			pwmRight = proportionalRight + integralTermRight + derivativeTermRight;

			// Limit the PWM values to a valid range (0 to 2048 in this case)
			pwmLeft = (pwmLeft > 1900) ? 1900 : (pwmLeft < 0 ? 0 : pwmLeft);
			pwmRight = (pwmRight > 2048) ? 2048 : (pwmRight < 0 ? 0 : pwmRight);

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmLeft);  // Adjust left motor PWM
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmRight); // Adjust right motor PWM

			// Check if the robot has reached the target distance
			if (encoderCount >= targetTicks) {
				// Stop the motor
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
				return;
			}
			htim1.Instance -> CCR4 = 155;
			break;
        case 'X':
        	htim1.Instance -> CCR4 == 90;
        	numRevolutions = 50 / (int)wheelCircumference;
        	targetTicks = (uint32_t)(numRevolutions * TICKS_PER_REV);
        	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);   // Left motor backward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);   // Right motor backward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
			currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
			encoderCount = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ?
							(startEncoderCount - currentEncoderCount) :
							(currentEncoderCount - startEncoderCount);

			// PID control calculations
			currentTime = HAL_GetTick();
			dt = (currentTime - previousTime) / 1000.0f;  // Calculate time difference in seconds
			previousTime = currentTime;

			errorLeft = targetTicks - encoderCount;  // Assume symmetric errors for simplicity
			errorRight = errorLeft;

			// Proportional term
			proportionalLeft = Kp * errorLeft;
			proportionalRight = Kp * errorRight;

			// Integral term (accumulate error over time)
			integralLeft += errorLeft * dt;
			integralRight += errorRight * dt;
			integralTermLeft = Ki * integralLeft;
			integralTermRight = Ki * integralRight;

			// Derivative term (rate of change of error)
			derivativeLeft = (errorLeft - previousErrorLeft) / dt;
			derivativeRight = (errorRight - previousErrorRight) / dt;
			derivativeTermLeft = Kd * derivativeLeft;
			derivativeTermRight = Kd * derivativeRight;

			// Update the previous errors for the next iteration
			previousErrorLeft = errorLeft;
			previousErrorRight = errorRight;

			// Calculate the final PWM adjustment using the PID formula
			pwmLeft = proportionalLeft + integralTermLeft + derivativeTermLeft;
			pwmRight = proportionalRight + integralTermRight + derivativeTermRight;

			// Limit the PWM values to a valid range (0 to 2048 in this case)
			pwmLeft = (pwmLeft > 1900) ? 1900 : (pwmLeft < 0 ? 0 : pwmLeft);
			pwmRight = (pwmRight > 2048) ? 2048 : (pwmRight < 0 ? 0 : pwmRight);

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmLeft);  // Adjust left motor PWM
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmRight); // Adjust right motor PWM

			// Check if the robot has reached the target distance
			if (encoderCount >= targetTicks) {
				// Stop the motor
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
				return;
			}
			htim1.Instance -> CCR4 = 155;
			break;
        case 'Y':
        	htim1.Instance -> CCR4 == 230;
			numRevolutions = 50 / (int)wheelCircumference;
			targetTicks = (uint32_t)(numRevolutions * TICKS_PER_REV);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);   // Left motor backward
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);   // Right motor backward
			HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
			currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
			encoderCount = (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) ?
							(startEncoderCount - currentEncoderCount) :
							(currentEncoderCount - startEncoderCount);

			// PID control calculations
			currentTime = HAL_GetTick();
			dt = (currentTime - previousTime) / 1000.0f;  // Calculate time difference in seconds
			previousTime = currentTime;

			errorLeft = targetTicks - encoderCount;  // Assume symmetric errors for simplicity
			errorRight = errorLeft;

			// Proportional term
			proportionalLeft = Kp * errorLeft;
			proportionalRight = Kp * errorRight;

			// Integral term (accumulate error over time)
			integralLeft += errorLeft * dt;
			integralRight += errorRight * dt;
			integralTermLeft = Ki * integralLeft;
			integralTermRight = Ki * integralRight;

			// Derivative term (rate of change of error)
			derivativeLeft = (errorLeft - previousErrorLeft) / dt;
			derivativeRight = (errorRight - previousErrorRight) / dt;
			derivativeTermLeft = Kd * derivativeLeft;
			derivativeTermRight = Kd * derivativeRight;

			// Update the previous errors for the next iteration
			previousErrorLeft = errorLeft;
			previousErrorRight = errorRight;

			// Calculate the final PWM adjustment using the PID formula
			pwmLeft = proportionalLeft + integralTermLeft + derivativeTermLeft;
			pwmRight = proportionalRight + integralTermRight + derivativeTermRight;

			// Limit the PWM values to a valid range (0 to 2048 in this case)
			pwmLeft = (pwmLeft > 1900) ? 1900 : (pwmLeft < 0 ? 0 : pwmLeft);
			pwmRight = (pwmRight > 2048) ? 2048 : (pwmRight < 0 ? 0 : pwmRight);

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmLeft);  // Adjust left motor PWM
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmRight); // Adjust right motor PWM

			// Check if the robot has reached the target distance
			if (encoderCount >= targetTicks) {
				// Stop the motor
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
				return;
			}
			htim1.Instance -> CCR4 = 155;
			break;

        default:
            // Stop the motors if no valid command
            HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
            OLED_ShowString(10, 10, "Invalid Command");
            return;  // Exit function
        }

        osDelay(1000);  // Adjust delay to give time for motors to respond
    }
    // USER CODE END motors
}

/* USER CODE END Header_motors */

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the Encoder thread.
* @param argument: Not used
* @retval None
*/
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  int cnt1, cnt2, diff, dir;
  char buffer[20];
  uint32_t tick;
  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
  tick = HAL_GetTick();

  for(;;)
  {
	memset(buffer, 0, sizeof buffer);
    if (HAL_GetTick() - tick > 1000L)
    {
      cnt2 = __HAL_TIM_GET_COUNTER(&htim3);
      if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
        if (cnt2 < cnt1) {
          diff = cnt1 - cnt2;
        } else {
          diff = (65535 - cnt2) + cnt1;
        }
      }
      else {
        if (cnt2 > cnt1) {
          diff = cnt2 - cnt1;
        } else {
          diff = (65535 - cnt1) + cnt2;
        }
      }

      //sprintf(buffer, "Speed:%5d", diff);
      //OLED_ShowString(10, 20, buffer);

      dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
      //sprintf(buffer, "Count1:%5d", cnt1);
      //OLED_ShowString(10, 30, buffer);


      // Refresh OLED after displaying
      OLED_Refresh_Gram();



      cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
      tick = HAL_GetTick();
    }

    //osDelay(500);  // Adjust delay to give enough time for the OLED to update.
  }
  /* USER CODE END encoder */
}
/* USER CODE END Header_encoder */

/* USER CODE BEGIN Header_Uart_Function */
/**
* @brief Function implementing the UART_Thread thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Uart_Function */
void Uart_Function(void *argument)
{
  /* USER CODE BEGIN Uart_Function */
  /* Infinite loop */
	HAL_UART_Receive_IT(&huart3,rxBuffer,5);
	//backward(100);
	//osDelay(5000);
//	frontRight(0);
//	osDelay(2000);
//	frontLeft(0);
//	osDelay(2000);
//	backRight(0);
//	osDelay(2000);
//	backLeft(0);
//	osDelay(2000);
  for(;;)
  {
//      char buf[8];
//      //HAL_UART_Receive_IT(&huart3,aRxBuffer,5);
//      OLED_Clear();
//	  OLED_ShowString(10,40,instructionBuffer);
//      OLED_Refresh_Gram();

      if(startFlag) {
    	  uint32_t iBufferSize,currentIndex,dirIndex,turnIndex,hunIndex,tensIndex,onesIndex,distance;
    	  int digit;
    	  char num[4];
    	  iBufferSize = instructionIndex + 1;
    	  currentIndex = 1;
    	  //OLED_ShowString(10,50,"start movement");

    	  while(currentIndex<iBufferSize){
    		  dirIndex = currentIndex;
    		  turnIndex = currentIndex + 1;
    		  hunIndex = currentIndex + 2;
    		  tensIndex = currentIndex + 3;
    		  onesIndex = currentIndex + 4;
        	  //char sbuf[10];
//        	  char sibuf[8];
//			  sprintf(sibuf,"size = %d",iBufferSize);
//			  OLED_ShowString(10,10,sibuf);
////        	  sprintf(sbuf,"%d %d %d %d %d",dirIndex,turnIndex,hunIndex,tensIndex,onesIndex);
////        	  OLED_ShowString(10,20,sbuf); //Note index starts from 1
//        	  osDelay(3000);

        	  //unpack distance

//        	  num[0] = instructionBuffer[hunIndex];
//        	  num[1] = instructionBuffer[tensIndex];
//        	  num[3] = instructionBuffer[onesIndex];
//        	  num[4] ='\0';
//


        	  int length = 3;
        	  strncpy(num, instructionBuffer + hunIndex-1, length);
        	  num[length] = '\0'; // Null-terminate the extracted

        	  digit = atoi(num);
        	  distance = (uint32_t)digit;

        	  char dist[8];
        	  sprintf(dist,"dist = %d",distance);
        	  OLED_ShowString(10,30,num);
        	  OLED_ShowString(10,50,dist); //Note index starts from 1
        	  osDelay(2000);

        	  char direction[3];
        	  direction[0] = instructionBuffer[dirIndex-1];
        	  direction[1] = instructionBuffer[turnIndex-1];
        	  OLED_Clear();
        	  OLED_ShowString(10,10,direction);
        	  osDelay(2000);
        	  if(instructionBuffer[dirIndex-1] =='F'&& instructionBuffer[turnIndex-1]== 'F'){
        		  OLED_Clear();
        		  OLED_ShowString(10,10,"Front Movement");
        		  forward(digit);

        	  }
        	  else if(instructionBuffer[dirIndex-1] =='R'&& instructionBuffer[turnIndex-1]== 'R'){
        		  backward(digit);
        	  }
        	  else if(instructionBuffer[dirIndex-1] =='F'&& instructionBuffer[turnIndex-1]== 'R'){
        		  frontRight(0);
        	  }
        	  else if(instructionBuffer[dirIndex-1] =='F'&& instructionBuffer[turnIndex-1]== 'L'){
        		  frontLeft(0);
        	  }
        	  else if(instructionBuffer[dirIndex-1] =='B'&& instructionBuffer[turnIndex-1]== 'R'){
          		  backRight(0);
        	  }
        	  else if(instructionBuffer[dirIndex-1] =='B'&& instructionBuffer[turnIndex-1]== 'L'){
        		  backLeft(0);
        	  }
        	  else if(instructionBuffer[dirIndex-1] =='S'&& instructionBuffer[turnIndex-1]== 'T'){
        		  //transmit 's' to rpi to confirm we have stopped so it can snap picture
        		  rpiBuffer[0] = 'S';
        		  rpiBuffer[1] = 'T';
        		  rpiBuffer[2] = '0';
        		  rpiBuffer[3] = '0';
        		  rpiBuffer[4] = '0';
        		  HAL_UART_Transmit(&huart3, rpiBuffer, 5, 1000);
        		  osDelay(1000);
        	  }

    		  currentIndex +=5;
    	  }
    	  if(currentIndex>=iBufferSize){
    		  OLED_Clear();
    		  OLED_ShowString(10,10,"exit forloop");
    		  break;
    	  }
      }

//      if (uartReceived == 0) {
//    	  OLED_Clear();
////          sprintf(buf, "Flag = %d", uartReceived);
////          OLED_ShowString(10, 10, buf);
////          OLED_ShowString(10, 30, "IN UART");
//
//
//
//      }
//      else if (uartReceived == 1) {
//    	  OLED_Clear();
//          sprintf(buf, "Flag = %d", uartReceived);
//          OLED_ShowString(10, 10, buf);
//          OLED_ShowString(10, 30, "Exit UART");
//          osDelay(3000);
//          uartReceived = 0;  // Re-enable UART reception
//
//      }

      osDelay(1000);  // Adjust delay as needed
  }
  /* USER CODE END Uart_Function */
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
