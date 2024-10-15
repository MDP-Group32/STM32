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
#include <math.h>
//#include "ICM20948.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

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
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UART_Thread */
osThreadId_t UART_ThreadHandle;
const osThreadAttr_t UART_Thread_attributes = {
  .name = "UART_Thread",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Gyroscope */
osThreadId_t GyroscopeHandle;
const osThreadAttr_t Gyroscope_attributes = {
  .name = "Gyroscope",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Encoder */
osThreadId_t EncoderHandle;
const osThreadAttr_t Encoder_attributes = {
  .name = "Encoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
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
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void Uart_Function(void *argument);
void GyroSensor(void *argument);
void encoder(void *argument);

/* USER CODE BEGIN PFP */
uint32_t Size;      // Size-point average, Size=1 to 512
uint32_t x[1024];   // two copies of MACQ
uint32_t *Pt;       // pointer to current
uint32_t I1;        // index to oldest
uint32_t LPFSum;    // sum of the last Size samples

//Variables related to imu (gyroscope and accelerometer)
//float gyroZ; // z axis only
//float gyroZ_bias; //z axis only
//uint32_t millisOld, millisNow; // time value
//float dt;// time elapse
//const uint8_t GYRO_SENS = GYRO_FULL_SCALE_2000DPS;
//const uint8_t ACCEL_SENS = ACCEL_FULL_SCALE_2G;

//Kalman filter variables
//float yaw_g_var = 2*2;
//float Var_gyro = 4*4;
//float yaw_kalman;


#define WHEEL_DIAMETER_CM 6.5
#define PI 3.14159
#define TICKS_PER_REV 360
#define MOTOR_PWM_MAX 6000  // Example maximum PWM value
#define MOTOR_PWM_MIN 0
#define MIN_PWM_THRESHOLD 100
#define I2C_ADDR 0

//IMU registers
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define REG_BANK_SEL 0x7F
#define IMU_ADDR	(0x68 << 1)
#define INT_PIN_CFG 0x0F
#define GYRO_CONFIG_1	(0x01)
#define GYRO_CONFIG_2  (0x02)
#define ACCEL_CONFIG	(0x14)
#define ACCEL_CONFIG_2  (0x15)
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2	0x07
#define LP_CONFIG	0x05
#define REG_WHO_AM_I 0x00
#define USER_CTRL  0x03
#define GYRO_SMPLRT_DIV 0x00
#define I2C_SLV0_ADDR 0x03
#define I2C_SLV0_REG 0x04
#define I2C_SLV0_CTRL 0x05
#define I2C_SLV0_DO 0x06
#define I2C_MST_CTRL 0x02
#define AK09916_ADDR (0x0C)
#define MAG_DEVICE_ID    0x01
#define MAG_STATUS1      0x10
#define MAG_DATA_ONSET   0x11
#define MAG_STATUS2      0x18
#define MAG_CTRL2	     0x31
#define MAG_CTRL3	     0x32
#define MAG_X_OUT 		 0x11
#define EXT_SLV_SENS_DATA_00 0x3B

//Command Transmission related User Defined Function Prototype and Variable
int startFlag = 0;
int bufferSize = 0;
uint8_t sizeBuffer[4] = "0000";
uint8_t *instructionBuffer; //point that points to character array
//uint8_t instructionBuffer[1000];
uint32_t instructionIndex = 0;
uint8_t rpiBuffer[5];


//Ultrasonic related User Defined Function Prototype and Variables
void UltraSonic(void);
double detectedDistance;


//IR Sensor related Functions Prototype and Variables
int32_t ConvertRDist(int32_t nc);
int32_t ConvertLDist(int32_t nc);
void IRSensor();
volatile uint32_t IRLeftDist, IRRightDist; //Variables to store distance from left and right of IR sensors


//ICM related User defined Function prototype and Variables
void ICM20948_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef __ICM20948_SelectUserBank(I2C_HandleTypeDef * hi2c, int userBankNum);
HAL_StatusTypeDef ICM_WriteByte(I2C_HandleTypeDef * hi2c, uint8_t const registerAddress, uint8_t writeData);
HAL_StatusTypeDef ICM_ReadByte(I2C_HandleTypeDef * hi2c, uint8_t const registerAddress, uint8_t * readData);
HAL_StatusTypeDef ICM_BrustRead(I2C_HandleTypeDef *hi2c, uint8_t const startAddress, uint16_t const amountOfRegistersToRead,uint8_t * readData);
static int16_t lPwmVal = 0, rPwmVal = 0;
static uint32_t time_elapsed = 0x00;
static uint32_t prev_time_elapsed = 0x00;
static uint32_t time_difference = 0x00;
float yawAngle;


// PID and Encoder related User Defined Structures, Function Prototype and variables
typedef struct{
	float Kp;
	float Ki;
	float Kd;

	float integral;
	float previousError;
}PID_TypeDef;

PID_TypeDef pid;

float PID_Compute(PID_TypeDef *pid, float setpoint, float measurement, int direction);
void PID_Reset(PID_TypeDef *pid);
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd);
uint32_t target_counts;
int rightEncoderVal = 0;



//Movement Related User Defined Functions
void forward(int distance);
void backward(int distance);
void stopMove();
void frontRight(int distance);
void backRight(int distance);
void frontLeft(int distance);
void backLeft(int distance);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//uint8_t instructBuffer[1000];

//uint8_t rxBuffer[5];
//uint8_t aRxBuffer[5];
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
//  IMU_init();

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
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_Thread */
  UART_ThreadHandle = osThreadNew(Uart_Function, NULL, &UART_Thread_attributes);

  /* creation of Gyroscope */
  GyroscopeHandle = osThreadNew(GyroSensor, NULL, &Gyroscope_attributes);

  /* creation of Encoder */
  EncoderHandle = osThreadNew(encoder, NULL, &Encoder_attributes);

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
  sConfig.Channel = ADC_CHANNEL_11;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Convert string recieved in sizeBuffer to integer
	if(bufferSize == 0 && startFlag == 0){
		//HAL_UART_Transmit(&huart3, &sizeBuffer, 4, 1000);
		bufferSize = atoi(sizeBuffer);
		instructionIndex = bufferSize;
		instructionBuffer = (uint8_t*)malloc(bufferSize * sizeof(uint8_t)); //Allocate memory dynamically for instructionBuffer
		HAL_UART_Receive_IT(&huart3, instructionBuffer, bufferSize);
	}
	else{
		startFlag = 1;
		HAL_UART_Transmit(&huart3, instructionBuffer, bufferSize, 1000);
	}
//    if(rxBuffer[0]!='0000'){
//    	if(instructionIndex<999){
//    		instructionBuffer[instructionIndex] = rxBuffer[0];
//    		instructionBuffer[++instructionIndex] = rxBuffer[1];
//    		instructionBuffer[++instructionIndex] = rxBuffer[2];
//    		instructionBuffer[++instructionIndex] = rxBuffer[3];
//    		instructionBuffer[++instructionIndex] = rxBuffer[4];
//    		instructionIndex++; //so that index start from 0
//    		HAL_UART_Receive_IT(&huart3, rxBuffer, 5);
//    	}
//    	else{
//    		OLED_Clear();
//    		OLED_ShowString(10,30,"bufferOverflow");
//    	}
//    }
//    else{
//    	startFlag = 1;
//    }
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


void LPF_Init(uint32_t initial, uint32_t size){ int i;
  if(size>1024) size=1024; // max
  Size = size;
  I1 = Size-1;
  LPFSum = Size*initial; // prime MACQ with initial data
  for(i=0; i<Size; i++){
    x[i] = initial;
  }
}

uint32_t LPF_Calc(uint32_t newdata){
  LPFSum = LPFSum+newdata-x[I1];   // subtract oldest, add newest
  x[I1] = newdata;     // save new data
  if(I1 == 0){
    I1 = Size-1;              // wrap
  } else{
    I1--;                     // make room for data
  }
  return LPFSum/Size;
}

// Convert smooth voltage value from LPF into distance
int32_t VoltToDist(int32_t nc){   // returns center distance in cm
  // write this for Lab 4
    uint32_t length;// distance in mm

    length = 12.44904 + (9940765 - 12.44904)/(1 + pow(nc/1.453177, 1.291693));

    return (length*10);
}

int32_t ConvertLDist(int32_t nc){
	uint32_t length; //distance in cm
	length = 28473.9/ (nc - 135.71);
	return length;
}

int32_t ConvertRDist(int32_t nc){
    uint32_t length;   // Distance in cm
    length = 29299/(nc - 137.02);
    return length;
}


void IRSensor() {
    // Read left sensor data from ADC (example for left sensor ADC1)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adc_left = HAL_ADC_GetValue(&hadc1);  // Read left sensor value
        //uint32_t filtered_left = LPF_Calc(adc_left);   // Apply low-pass filter
        IRLeftDist = ConvertLDist(adc_left); // Convert to distance (cm)
    }

    // Read right sensor data from ADC (example for right sensor ADC2)
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
        uint32_t adc_right = HAL_ADC_GetValue(&hadc2);  // Read right sensor value
        //uint32_t filtered_right = LPF_Calc(adc_right);  // Apply low-pass filter
        IRRightDist = ConvertRDist(adc_right); // Convert to distance (cm)

    }
}
//IMU related functions
// IMU related functions
HAL_StatusTypeDef ICM_WriteByte(I2C_HandleTypeDef * hi2c, uint8_t const registerAddress, uint8_t writeData){
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(
							hi2c,
							IMU_ADDR,
							registerAddress,
							I2C_MEMADD_SIZE_8BIT,
							&writeData,
							I2C_MEMADD_SIZE_8BIT,
							10);
	return status;
}

HAL_StatusTypeDef ICM_ReadByte(I2C_HandleTypeDef * hi2c, uint8_t const registerAddress, uint8_t * readData){
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(
							hi2c,
							IMU_ADDR,
							registerAddress,
							I2C_MEMADD_SIZE_8BIT,
							readData,
							I2C_MEMADD_SIZE_8BIT,
							10);
	return status;
}

HAL_StatusTypeDef ICM_BrustRead(I2C_HandleTypeDef *hi2c, uint8_t const startAddress, uint16_t const amountOfRegistersToRead,uint8_t * readData){
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(
							hi2c,
							IMU_ADDR,
							startAddress,
							I2C_MEMADD_SIZE_8BIT,
							readData,
							amountOfRegistersToRead * I2C_MEMADD_SIZE_8BIT,
							10);
	return status;

}

HAL_StatusTypeDef ICM20948_SelectUserBank(I2C_HandleTypeDef * hi2c, int userBankNum) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t writeData = userBankNum << 4;

	status = HAL_I2C_Mem_Write(
			hi2c,
			IMU_ADDR,
			REG_BANK_SEL,
			I2C_MEMADD_SIZE_8BIT,
			&writeData,
			I2C_MEMADD_SIZE_8BIT,
			10);

	return status;
}

void __ICM20948_init(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef status = HAL_OK;

	//Bank 0
	status = ICM20948_SelectUserBank(hi2c, 0x0);
	//Power Reset
	status = ICM_WriteByte(hi2c, PWR_MGMT_1, 0x80);
	HAL_Delay(200);
	status = ICM_WriteByte(hi2c, PWR_MGMT_1, 0x01);

	//Enable both gyroscope and accelerometer
	status = ICM_WriteByte(hi2c, PWR_MGMT_2, 0x38);

	//Bank 2
	status = ICM20948_SelectUserBank(hi2c, 0x2);

	status = ICM_WriteByte(hi2c, GYRO_CONFIG_1, 0x39);

	status = ICM_WriteByte(hi2c, GYRO_SMPLRT_DIV, 0x08);

	status = ICM_WriteByte(hi2c, ACCEL_CONFIG, 0x39);

	//Bank 0
	status = ICM20948_SelectUserBank(hi2c, 0x0);
	//Interrupt pin config: BYPASS ENABLE
	status = ICM_WriteByte(hi2c, 0x0F, 0x02);


}

uint32_t countTargetTicks(int distance) {
    float rev, dist, circumference;
    uint32_t targetTicks;

    dist = (float)distance;  // Convert distance to float
    circumference = PI * WHEEL_DIAMETER_CM;  // Calculate the wheel's circumference in cm

    rev = dist / circumference;  // Calculate the number of wheel revolutions needed
    targetTicks = (uint32_t)(rev * TICKS_PER_REV);  // Convert revolutions to encoder ticks (adjust factor if needed)

    return targetTicks;
}

// Function to handle PID motor control
int readLeftEncoder() {
    int cntL = __HAL_TIM_GET_COUNTER(&htim2);  // Read the left encoder value
    int diffL;
    int dirL = 1;

    if (cntL > 32000) {
        dirL = 1;
        diffL = (65536 - cntL);
    } else {
        dirL = -1;
        diffL = cntL;
    }

    // Update left encoder value based on direction
    int leftEncoderVal = 0;
    if (dirL == 1) {
        leftEncoderVal -= diffL;
    } else {
        leftEncoderVal += diffL;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset the counter after reading
    return leftEncoderVal;
}

int readRightEncoder() {
    int cntR = __HAL_TIM_GET_COUNTER(&htim3);  // Read the right encoder value
    int diffR;
    int dirR = 1;

    if (cntR > 32000) {
        dirR = 1;
        diffR = (65536 - cntR);
    } else {
        dirR = -1;
        diffR = cntR;
    }

    // Update right encoder value based on direction
    int rightEncoderVal = 0;
    if (dirR == 1) {
        rightEncoderVal -= diffR;
    } else {
        rightEncoderVal += diffR;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0);  // Reset the counter after reading
    return rightEncoderVal;
}

float PID_Compute(PID_TypeDef *pid, float setpoint, float measurement, int direction){
	float error;
	if (direction == 0){
		error = setpoint - measurement;
	}
	else{
		error = measurement-setpoint;
	}
	pid->integral += error;
	float derivative = error - pid->previousError;
	pid->previousError = error;
	float output = ((pid->Kp*error) + (pid->Ki * pid->integral) + (pid->Kd * derivative));
	output = 2000 +(5000*output/100.0);
	return output;
}

void PID_Reset(PID_TypeDef *pid){
	pid->integral = 0.0f;
	pid->previousError = 0.0f;
}

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd){
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;

	pid->integral = 0.0f;
	pid->previousError = 0.0f;
}

void forward(int distance){
	yawAngle = 0.0;
	htim1.Instance->CCR4 = 155;
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);


    double targetTicks = countTargetTicks(distance);

    int startCountB = __HAL_TIM_GET_COUNTER(&htim3);
    int encoderCountB;
    int currentCountB ;

    PID_Init(&pid, 9.8f, 0.01f, 0.05f);

    while (1) {

    	currentCountB = __HAL_TIM_GET_COUNTER(&htim3);
    	encoderCountB = abs(currentCountB - startCountB);
    	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 2500);
    	uint16_t pwmValue = PID_Compute(&pid,0,yawAngle,0);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValue);


        if (rightEncoderVal >= targetTicks) {
            stopMove();
            break;
        }

        osDelay(10);
    }
}

void backward(int distance){
	yawAngle = 0.0;
	htim1.Instance->CCR4 = 150;
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);


	double targetTicks = countTargetTicks(distance);

	int startCountB = __HAL_TIM_GET_COUNTER(&htim3);
	int encoderCountB;
	int currentCountB ;

	PID_Init(&pid, 9.0f, 0.01f, 0.05f);

	while (1) {
		currentCountB = __HAL_TIM_GET_COUNTER(&htim3);
		encoderCountB = abs(currentCountB - startCountB);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,2500);
		uint16_t pwmValue = PID_Compute(&pid,0,yawAngle,1);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValue);


		if (abs(rightEncoderVal) >= targetTicks) {
			stopMove();
			break;
		}

		osDelay(10);
	}
}

void stopMove(){
    // Reset GPIO pins to stop motors
    HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

    // Set PWM values to zero to fully stop motors
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
}

void frontRight(int distance){
    yawAngle = 0.0; //reset angle
    uint32_t pwmValL = 4200;
    uint32_t pwmValR = 500;
    uint32_t encoderCount = 0;
    uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t currentEncoderCount = 0;
    uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

        //(uint32_t)countTargetTicks(distance);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 260;
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

    for(;;){
      //Move Forward
       HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
       HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValL);
       HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValR);
      currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
      osDelay(10);
        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (fabs(yawAngle)>=69) {
              // Stop the motor
            HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
            HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
            htim1.Instance -> CCR4 = 155;
            yawAngle = 0.0;

              return;
          }
    }


}


void backRight(int distance){
	yawAngle = 0.0; //reset angle
    uint32_t pwmValL = 4300;
    uint32_t pwmValR = 500;
  	uint32_t encoderCount = 0;
  	uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
  	uint32_t currentEncoderCount = 0;
  	uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

  			//(uint32_t)countTargetTicks(distance);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 260;
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

  	for(;;){
  	  //Move Backwards
  		 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
  		 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValL);
  		 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
  		 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
  		 __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValR);
  		osDelay(10);
  		currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (fabs(yawAngle)>= 69) {
              // Stop the motor
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              htim1.Instance -> CCR4 = 155;
              yawAngle = 0.0;
              return;
          }
  	}
}
void frontLeft(int distance){
	yawAngle = 0.0; //reset angle
    uint32_t pwmValL = 800;
    uint32_t pwmValR = 4000;
    uint32_t encoderCount = 0;
    uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t currentEncoderCount = 0;
    uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 120;
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);


    for(;;){
      //Move Forward
       HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
       HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValL);
       HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValR);
      osDelay(10);
      currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (fabs(yawAngle)>= 86) {
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              htim1.Instance -> CCR4 = 155;
              yawAngle = 0.0;
              return;
          }
    }

}

void backLeft(int distance){
	yawAngle = 0.0; //reset angle
    uint32_t pwmValL = 950;
    uint32_t pwmValR = 3600;
    uint32_t encoderCount = 0;
    uint32_t startEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t currentEncoderCount = 0;
    uint32_t targetTicks = (uint32_t)countTargetTicks(distance);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    htim1.Instance->CCR4 = 88;
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);


    for(;;){
      //Move Backwards
       HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
       HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwmValL);
       HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);
       HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
       __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwmValR);
      osDelay(10);
      currentEncoderCount = __HAL_TIM_GET_COUNTER(&htim2);

        // Check if the robot has reached the target distance

          // Calculate total encoder ticks since the motor started moving
          if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
              encoderCount = (startEncoderCount - currentEncoderCount);
          } else {
              encoderCount = (currentEncoderCount - startEncoderCount);
          }

          if (fabs(yawAngle)>=70) {
              HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);  // Stop PWM
              HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
              __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);  // Stop PWM
              htim1.Instance -> CCR4 = 155;
              yawAngle = 0.0;
              return;
          }
    }
}

void resetAllGlobals() {
    target_counts = 0;
    detectedDistance = 0.0;
    startFlag = 0;
    //memset(rxBuffer, 0, sizeof(rxBuffer));
    //memset(rpiBuffer, 0, sizeof(rpiBuffer));
    memset(instructionBuffer, 0, sizeof(instructionBuffer));
    instructionIndex = 0;
}

/*
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
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}

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
//	osDelay(500);
//	backLeft(0);
//	osDelay(1000000);
	// frontRight(0);
	// resetAllGlobals();
	// osDelay(5000);
	// backRight(0);
	// osDelay(5000);
	// frontLeft(0);
	// osDelay(5000);
	// backLeft(0);
	// osDelay(5000);
	HAL_UART_Receive_IT(&huart3,sizeBuffer ,4);
	  for(;;)
	  {
	      if(startFlag) {
	       uint32_t iBufferSize,currentIndex,dirIndex,turnIndex,hunIndex,tensIndex,onesIndex,distance;
	       int digit;
	       char num[4];
	       iBufferSize = instructionIndex;
	       currentIndex = 0;
	       //OLED_ShowString(10,50,"start movement");

	       while(currentIndex<iBufferSize){
	        dirIndex = currentIndex;
	        turnIndex = currentIndex + 1;
	        hunIndex = currentIndex + 2;
	        tensIndex = currentIndex + 3;
	        onesIndex = currentIndex + 4;

	           int length = 3;
	           strncpy(num, instructionBuffer + hunIndex, length);
	           num[length] = '\0'; // Null-terminate the extracted

	           digit = atoi(num);
	           distance = (uint32_t)digit;

	           char dist[8];
	           osDelay(2000);
//	           sprintf(dist,"dist = %d",distance);
//	           OLED_ShowString(10,30,num);
//	           OLED_ShowString(10,50,dist); //Note index starts from 1
//	           osDelay(500);

	           char direction[3];
	           direction[0] = instructionBuffer[dirIndex];
	           direction[1] = instructionBuffer[turnIndex];
	           osDelay(2000);
//	           OLED_Clear();
//	           OLED_ShowString(10,10,direction);
//	           osDelay(500);
	           if(instructionBuffer[dirIndex] =='F'&& instructionBuffer[turnIndex]== 'F'){
//	            OLED_Clear();
//	            OLED_ShowString(10,10,"Front Movement");
	        	osDelay(500);
	            forward(digit);
	            osDelay(3000);

	           }
	           else if(instructionBuffer[dirIndex] =='R'&& instructionBuffer[turnIndex]== 'R'){
	        	osDelay(500);
	            backward(digit);
	            osDelay(3000);
	           }
	           else if(instructionBuffer[dirIndex] =='F'&& instructionBuffer[turnIndex]== 'R'){
	        	osDelay(500);
	            frontRight(0);
	            osDelay(3000);
	           }
	           else if(instructionBuffer[dirIndex] =='F'&& instructionBuffer[turnIndex]== 'L'){
	        	osDelay(50);
	            frontLeft(0);
	            osDelay(3000);
	           }
	           else if(instructionBuffer[dirIndex] =='B'&& instructionBuffer[turnIndex]== 'R'){
	        	osDelay(500);
	            backRight(0);
	            osDelay(3000);
	           }
	           else if(instructionBuffer[dirIndex] =='B'&& instructionBuffer[turnIndex]== 'L'){
	        	osDelay(500);
	            backLeft(0);
	            osDelay(3000);
	           }
	           else if(instructionBuffer[dirIndex] =='S'&& instructionBuffer[turnIndex]== 'T'){
	            //transmit 's' to rpi to confirm we have stopped so it can snap picture
	            HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	            HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	            rpiBuffer[0] = instructionBuffer[dirIndex];
	            rpiBuffer[1] = instructionBuffer[turnIndex];
	            rpiBuffer[2] = instructionBuffer[hunIndex];
	            rpiBuffer[3] = instructionBuffer[tensIndex];
	            rpiBuffer[4] = instructionBuffer[onesIndex];
	            HAL_UART_Transmit(&huart3, rpiBuffer, 5, 1000);
	            osDelay(1000);
	           }

	        currentIndex +=5;
	       }
	       if(currentIndex>=iBufferSize){
	        OLED_Clear();
	        OLED_ShowString(10,10,"exit forloop");
	        resetAllGlobals();
	        break;
	       }
	      }
	//
	//      }

	      osDelay(1000);  // Adjust delay as needed
	  }
  /* USER CODE END Uart_Function */
}

/* USER CODE BEGIN Header_GyroSensor */
/**
* @brief Function implementing the Gyroscope thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GyroSensor */
void GyroSensor(void *argument)
{
  /* USER CODE BEGIN GyroSensor */
  /* Infinite loop */
	__ICM20948_init(&hi2c1);
	prev_time_elapsed = HAL_GetTick();
	ICM20948_init(&hi2c1);
	int16_t reading;
	uint8_t gyroZ[2];
	yawAngle = 0.0;
	float sum = 0.0;
	int printAngle;
	char angleBuffer[8];
	for (int i = 0; i < 100; i++) {
		ICM_BrustRead(&hi2c1, GYRO_ZOUT_H, 2, &gyroZ);
		reading = (int16_t)(gyroZ[0] << 8 | gyroZ[1]);
		sum += reading;
		osDelay(1); // Delay to simulate time between readings
	  }
	 float gyroOffset = sum / 100.0;

	for (;;)
	{
		OLED_ShowString(0,0, "GyroStart");
		OLED_Refresh_Gram();
		time_elapsed = HAL_GetTick();
		 time_difference = time_elapsed - prev_time_elapsed;
		 ICM_BrustRead(&hi2c1, GYRO_ZOUT_H, 2, &gyroZ);
		 reading = (int16_t)(gyroZ[0] << 8 |gyroZ[1]);
		 yawAngle += (float)((time_difference * (reading - gyroOffset)/ 131.0) )/ 1000.0;
		 //sprintf(oled_buf1,"%.2f\n\r", yawAngle);
		 //HAL_UART_Transmit(&huart3, (uint8_t*)oled_buf1, strlen(oled_buf1), HAL_MAX_DELAY); 	//serial plot
		 prev_time_elapsed = time_elapsed;
		 printAngle = (int)fabs(yawAngle);
		 sprintf(angleBuffer,"Angle = %d",printAngle);
		 OLED_ShowString(10,20,angleBuffer);
		 OLED_Refresh_Gram();
		 if(fabs(yawAngle) >= 90){
			 OLED_ShowString(10,10, "90");
			 OLED_Refresh_Gram();
		 }
		 osDelay(1);
	}
  /* USER CODE END GyroSensor */
}

/* USER CODE BEGIN Header_encoder */
/**
* @brief Function implementing the Encoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
  /* Infinite loop */
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	int cntR = __HAL_TIM_GET_COUNTER(&htim3);  // Read the right encoder value
	    int diffR;
	    int dirR = 1;
	    uint32_t tick = HAL_GetTick();

	   // Reset the counter after reading
  for(;;)
  {
	  if(HAL_GetTick()-tick > 10L){
	  int cntR = __HAL_TIM_GET_COUNTER(&htim3);
	  if (cntR > 32000) {
	  	        dirR = 1;
	  	        diffR = (65536 - cntR);
	  	    } else {
	  	        dirR = -1;
	  	        diffR = cntR;
	  	    }

	  	    // Update right encoder value based on direction

	  	    if (dirR == 1) {
	  	        rightEncoderVal -= diffR;
	  	    } else {
	  	        rightEncoderVal += diffR;
	  	    }

	  	    __HAL_TIM_SET_COUNTER(&htim3, 0);
	  }
    osDelay(1);
  }
  /* USER CODE END encoder */
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
