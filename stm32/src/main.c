/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "sensors.h"
#include "pidMotor.h"
#include "pidHeading.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ECHO_Port 	GPIOB
#define ECHO_Pin	GPIO_PIN_5
#define TRIG_Port	GPIOB
#define TRIG_Pin	GPIO_PIN_4

#define SERVOCENTER 147

#define SERVOMIN 94
#define SERVOMAX 232

//old value
//#define MOTORHIGH 4000
//#define MOTORMID 2200
//#define MOTORLOW 800

#define MOTORHIGH 6000 // 4000
#define MOTORMID 3000 //2600
#define MOTORLOW 800

#define MOTORMIDIR 4500 //4000
#define MOTORMIDTURN 4500 //4000

#define MOTORMAX 7000
#define MOTORMIN 0

#define PI 3.1415f

//ACCELERATION
#define DECELERATIONDIST 28 // 22
#define DECELERATIONRATE 350 //decrease 250 every 20ms

#define DECELERATIONBEARING 15

//Ultrasound
#define USMINDELAY 40
#define USACCERLERATIONMULTIPLIER 2

//Calibration stuff
#define MOTOR_PPR 1535.0f
#define TURNRADIUS 28.0f
#define TURNRATIO 0.50f //0.52f

#define SERVOCENTER 147

#define MOVEIROVERSHOOT 3.0f

/* Set to 1 to run a boot-time hardware test instead of normal operation --
 * sweeps the servo, then spins each motor forward/backward at low speed
 * while showing live encoder counts on the OLED. No UART/protocol
 * dependency, since USART3's real wiring hasn't been checked against the
 * schematic yet. Wheels off the ground before enabling this. */
#define HARDWARE_TEST_MODE 0

//===INDOOR CALIBRATION===
#define WHEEL_D_CM_INDOOR 6.47f
#define MOVEOVERSHOOT_INDOOR 1.5f//1.6f
#define TURNOVERSHOOT_INDOOR 1.2f//1.2

#define SERVOLEFT_INDOOR 99	//102
#define SERVORIGHT_INDOOR 227	//216

#define MOVEUSOVERSHOOT_INDOOR 3.2f

//===OUTDOOR CALIBRATION===
#define WHEEL_D_CM_OUTDOOR 6.47f
#define MOVEOVERSHOOT_OUTDOOR 1.5f//1.5f
#define TURNOVERSHOOT_OUTDOOR 1.2f//1.0f

#define SERVOLEFT_OUTDOOR 99	//102
#define SERVORIGHT_OUTDOOR 227	//228

#define MOVEUSOVERSHOOT_OUTDOOR 3.2f

//IR Related
#define DIST_IR_MIN 5.0f
#define DIST_IR_MAX 70.0f

#define DIST_IR_OBSTACLE 35.0f

//Task 2
#define T2ENTRYDIST 10.0f //20.0f
#define T2ROBOTOFFSET 20.0f
#define T2FIRSTUSDISTCAMERA 50 //distance of center of robot to the obstacle
#define T2SECONDUSDISTCAMERA 50 //
#define T2FIRSTBACKOFF 0 //distance to backoff before going around first obstacle
#define T2SECONDBACKOFF 0 //distance to backoff before going around second obstacle

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2; /* was hi2c1 -- IMU is really on I2C2 (PB10/PB11), see stm32f4xx_hal_msp.c */

TIM_HandleTypeDef htim1; /* right motor PWM (MOTORD, PE13/PE14) */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5; /* right motor encoder (MOTORD, PA0/PA1) */
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
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal3,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal4,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for oledUpdateTask */
osThreadId_t oledUpdateTaskHandle;
const osThreadAttr_t oledUpdateTask_attributes = {
  .name = "oledUpdateTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void *argument);
void motor(void *argument);
void encoder_task(void *argument);
void comm_task(void *argument);
void gyro_task(void *argument);
void oled_update_task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//===CALIBRATION VARIABLES===
uint8_t indoorOutdoor=0; //0:indoor, 1:outdoor
float WHEEL_D_CM = WHEEL_D_CM_INDOOR;
float MOVEOVERSHOOT = MOVEOVERSHOOT_INDOOR;
float TURNOVERSHOOT = TURNOVERSHOOT_INDOOR;

int SERVOLEFT = SERVOLEFT_INDOOR;
int SERVORIGHT = SERVORIGHT_INDOOR;

float MOVEUSOVERSHOOT = MOVEUSOVERSHOOT_INDOOR;

// Communication
uint8_t aRxBuffer[5];
uint8_t instrList[40][5];
uint8_t instrLen=0;
uint8_t receivedInstruction = 0;

//Motor
int32_t right_target = 0, left_target = 0, dist_target = 0;
float right_dist = 0, left_dist = 0;
int8_t right_dir = 0, left_dir = 0; // -3:fast reverse, -2:reverse, -1:fast Reverse, 0:Stop, 1:slow forward, 2:forward, 3: fast forward
uint16_t pwmVal_motor_target = 0;
uint16_t right_pwmVal_motor = MOTORMID, left_pwmVal_motor = MOTORMID;
int8_t move_dir= 'C'; //C:Center, L:Left, R:Right
//Motor PID stuff
int motor_correction=0;
uint8_t motor_pid = 0;

//Servo
uint16_t target_pwmVal_servo = SERVOCENTER;
uint16_t pwmVal_servo = SERVOCENTER;


//Servo PID
int heading_correction=0;
uint8_t heading_pid = 0;


// Sensor value
Sensors sensors;
float heading_z=0;
float heading_driff;	//drift in degree per ms
float heading_target = 0;

// Ultrasonic Sensor
int echo = 0;
int tc1, tc2;
float usDist = 100; //in cm

// IR Sensor
float irDistL = 0; //in cm
float irDistR = 0; //in cm

//OLED
uint8_t oled_display[6][16];

//Coordinates
//int coord_x=199,coord_y=199;	// in cm
//int8_t coord_bearing = 0; //0:N E,S,W

//TASK 2 VARIABLES
int T2_Dist_y_A = 0;
int T2_Dist_y_B = 0;
//int T2_Dist_y_C = 0;
int T2_return_y = 0; //T2_return_y = T2_Dist_y_A + T2_Dist_y_B + 4*TURNRADIUS - T2ENTRYDIST - T2ROBOTOFFSET - T2FIRSTBACKOFF - T2SECONDBACKOFF

int T2_Dist_x_A = 0;
int T2_Dist_x_B = 0;
int T2_return_x = 0; //T2_return_x = T2_Dist_x_A - T2_Dist_x_B - 2*TURNRADIUS

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t *OLED_buf;


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
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* Init all my dear libraries */
  OLED_Init();
  sensors_init(&hi2c2, &sensors);
  motor_encoder_init(&htim2, &htim5); /* right encoder moved off TIM4 (now left motor PWM) to TIM5 -- see main.h and MX_TIM5_Init */

//  OLED_ShowString(0,5, "My Display");
  OLED_Refresh_Gram();

  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,5);

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

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(comm_task, NULL, &CommTask_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(gyro_task, NULL, &GyroTask_attributes);

  /* creation of oledUpdateTask */
  oledUpdateTaskHandle = osThreadNew(oled_update_task, NULL, &oledUpdateTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* Diagnostic 2026-08-25: osThreadNew()'s return is never checked anywhere
   * in this codebase. If any task failed to create, show which one on the
   * OLED immediately (before osKernelStart(), independent of any task
   * actually running) rather than silently continuing. */
  if (!defaultTaskHandle || !MotorTaskHandle || !EncoderTaskHandle ||
      !CommTaskHandle || !GyroTaskHandle || !oledUpdateTaskHandle)
  {
    sprintf((char *)oled_display[0], "NULL:%c%c%c%c%c%c   ",
            defaultTaskHandle ? '-' : 'D',
            MotorTaskHandle ? '-' : 'M',
            EncoderTaskHandle ? '-' : 'E',
            CommTaskHandle ? '-' : 'C',
            GyroTaskHandle ? '-' : 'G',
            oledUpdateTaskHandle ? '-' : 'O');
    OLED_ShowString(0, 0, oled_display[0]);
    OLED_Refresh_Gram();
  }
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
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_Delay(1000);
	  sprintf(oled_display[3], "NAWWWWWW");


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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
  ADC_InjectionConfTypeDef sConfigInjected = {0};

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

  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
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
static void MX_I2C2_Init(void)
{
  /* Renamed from MX_I2C1_Init 2026-08-25 -- the IMU is wired to I2C2
   * (PB10/PB11) on this board, not I2C1 (PB8/PB9, which is really the
   * left motor driver's IN1/IN2 here). See stm32f4xx_hal_msp.c. */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
/* Repurposed 2026-08-25: originally drove PE14 as "the servo" -- wrong,
 * PE14 is really MOTORD's IN1 (see TIM8/main.h). Now the right motor's PWM
 * (PE13/PE14, MOTORD's IN2/IN1). Same PSC/ARR as TIM4 (0/7199, ~2.2kHz),
 * the scale MOTORHIGH/MID/LOW/MAX were already tuned against. TIM1 is an
 * advanced-control timer, so (like TIM8) it needs a BreakDeadTime config
 * even for plain PWM output. (First tried TIM9/PE5/PE6 for MOTORB's
 * driver -- moved to MOTORD because MOTORB's encoder pins conflicted with
 * the ultrasonic sensor; see MX_TIM5_Init.) */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  htim2.Init.Period = 65535;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* Encoder mode briefly added here 2026-08-25 for a MOTORB-paired right
   * encoder, then removed: it conflicted with the ultrasonic capture above
   * (same timer). Right encoder moved to TIM5/MOTORD instead -- see
   * MX_TIM5_Init. This is purely the ultrasonic timer again. */
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* Repurposed 2026-08-25: was MOTORC's encoder (Encoder mode, TI12).
   * MOTORC isn't used any more -- this is now the left motor's PWM
   * (PB8/PB9 = U8's IN2/IN1, MOTORA). Same PSC/ARR as TIM1's PWM config
   * below (0/7199), see its comment for why. */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
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
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
/* Added 2026-08-25: right motor's encoder (MOTORD, PA0/PA1). Mirrors
 * TIM2's encoder config (TIM5 is also a 32-bit general-purpose timer). */
static void MX_TIM5_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

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
  htim6.Init.Prescaler = 15;
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
  /* Repurposed 2026-08-25: PC6/CH1 is the real servo pin on this board
   * (STM_Ref called it "MotorA_PWM" -- wrong, see main.h). PSC/ARR changed
   * from 0/7199 (~2.2kHz, was meant for motor PWM) to 160/1000, copied
   * from the old TIM1 servo config so SERVOMIN/CENTER/MAX keep meaning
   * what they already meant (~99Hz, CCR 0-1000 maps to roughly a
   * 0.95-2.3ms pulse). CH3/PC8 dropped -- unused on this board. */
  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 160;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE(); /* added for the corrected OLED pins */

  /* MotorA_AIN1/AIN2 and MotorB_CIN1/CIN2 removed 2026-08-25: those were
   * plain-GPIO direction pins on the wrong (STM_Ref-assumed) pins. The
   * real motor driver pins are PWM timer channels now (TIM4 CH3/CH4 for
   * the left motor, TIM1 CH3/CH4 for the right), configured in
   * HAL_TIM_Base_MspInit (stm32f4xx_hal_msp.c), not here. */

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level (OLED, corrected to GPIOD) */
  HAL_GPIO_WritePin(GPIOD, OLED_SCLK_Pin|OLED_SDA_Pin|OLED_RESET_Pin|OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level (Buzzer corrected to PA8, see main.h) */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, US_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SCLK_Pin OLED_SDA_Pin OLED_RESET_Pin OLED_DC_Pin
                           (corrected to GPIOD -- see main.h) */
  GPIO_InitStruct.Pin = OLED_SCLK_Pin|OLED_SDA_Pin|OLED_RESET_Pin|OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin (corrected to PA8, see main.h) */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US_Trigger_Pin */
  GPIO_InitStruct.Pin = US_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		// Prevent unused argument(s) compilation warning
		UNUSED(huart);
		HAL_UART_Receive_IT(&huart3, aRxBuffer, 5);

//		HAL_UART_Transmit(&huart3, (uint8_t *) "ACK\r\n", 5, 0xFFFF);
		if(instrLen>=40)
			HAL_UART_Transmit(&huart3, (uint8_t *) "FUL\r\n", 5, 0xFFFF);
		else if (receivedInstruction == 1)
			HAL_UART_Transmit(&huart3, (uint8_t *) "BUS\r\n", 5, 0xFFFF);
		{
			memcpy(instrList[instrLen], aRxBuffer, sizeof(aRxBuffer));
			instrLen++;
		}

		if(aRxBuffer[0]=='#')
		{
			receivedInstruction = 1;
			//sprintf(oled_display[0], "shld start");
			HAL_UART_Transmit(&huart3, (uint8_t *) "RUN\r\n", 5, 0xFFFF);
		}
		if(aRxBuffer[0]=='Q')
			EStop();

		sprintf(oled_display[3], "Rx:%s        ", aRxBuffer);
		//HAL_UART_Transmit(&huart3, (uint8_t *)aRxBuffer, 10, 0xFFFF);

	}

	// microsecond delay implemented with htim6
	void delay_us(uint16_t us){
		HAL_TIM_Base_Start(&htim6);
		__HAL_TIM_SET_COUNTER(&htim6, 0);

		while(__HAL_TIM_GET_COUNTER(&htim6) < us);
	}


	void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
	{
		if(htim==&htim3)
		{
			if(HAL_GPIO_ReadPin(ECHO_Port, ECHO_Pin) == GPIO_PIN_SET)
			{	//If pin on high, means positive edge
				tc1 = (int)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//Retrive value and store in tc1
			}
			else if (HAL_GPIO_ReadPin(ECHO_Port, ECHO_Pin) == GPIO_PIN_RESET)
			{	//If pin on low means negative edge
				tc2 = (int)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);	//Retrive val and store in tc2
				if (tc2 > tc1)
				{
					echo = tc2-tc1;		//Calculate the differnce = width of pulse
				}
				else
				{
					echo = (65536-tc1)+tc2;
				}
				usDist = 0.70*usDist + (1-0.70)*((echo * (343 /2.0)) / 10000.0); //WITH FILTER
				sprintf(oled_display[5], "usDist:%d   ", (int)usDist);
			}
		}
	}



	void EStop()
	{
		OLED_ShowString(0, 0, "E STOP          ");
		OLED_Refresh_Gram();
		while(1)
		{
			OLED_ShowString(0, 0, "E STOP          ");
			OLED_Refresh_Gram();
			/* Both direction channels to 0 duty -- motors stopped either way,
			 * see motor() below for the PWM-through-IN1/IN2 scheme. */
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
			__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

			htim8.Instance->CCR1 = SERVOCENTER;	//extreme right
		}
	}

	void FrontCenter(int dist)
	{
		int cur_dist=0;
		move_dir = 'C';
		target_pwmVal_servo = SERVOCENTER;
		reset_Encoders();
		reset_encoder_pid_error();
		reset_heading_pid_error();
		motor_pid = 1;
		heading_pid = 1;

		left_dist = 0;
		right_dist = 0;
		dist_target = dist;

		//acceleration start
		right_dir = 1;
		left_dir = 1;
		pwmVal_motor_target = MOTORLOW;
		if(dist_target<DECELERATIONDIST+2 && dist_target>5)//manual acceleration
		{
			osDelay(50);
			pwmVal_motor_target = MOTORMID;
			while(cur_dist<dist_target-8)
			{
				osDelay(20);
				cur_dist = (left_dist+right_dist)/2;
			}

		}
		else
		{
			//acceleration start
			while(pwmVal_motor_target<MOTORHIGH && cur_dist<dist_target-DECELERATIONDIST)
			{
				pwmVal_motor_target += DECELERATIONRATE;
				cur_dist = (left_dist+right_dist)/2;
				osDelay(20);
			}
			//acceleration end
			while(cur_dist<dist_target-DECELERATIONDIST)
			{
				pwmVal_motor_target = MOTORHIGH;
				osDelay(20);
				cur_dist = (left_dist+right_dist)/2;
			}
		}

		//Deceleration start
		while(pwmVal_motor_target>MOTORLOW && cur_dist<dist_target-MOVEOVERSHOOT)
		{
			pwmVal_motor_target -= DECELERATIONRATE;
			cur_dist = (left_dist+right_dist)/2;
			osDelay(20);
		}
		//Deceleration end
		while(cur_dist<dist_target-MOVEOVERSHOOT)
		{
			pwmVal_motor_target = MOTORLOW;
			cur_dist = (left_dist+right_dist)/2;
			osDelay(10);
		}

		motor_pid = 0;
		heading_pid = 0;

		right_dir = 0;
		left_dir = 0;
		pwmVal_motor_target = 0;

		target_pwmVal_servo = SERVOCENTER;

//		osDelay(50);
	}

	void FrontRight(int angle)
	{
		move_dir = 'R';
		motor_pid = 0;
		heading_pid = 0;

		target_pwmVal_servo = SERVORIGHT;
		heading_target = heading_target+angle;
		osDelay(100);
		right_dir = 1;
		left_dir = 1;
		pwmVal_motor_target = MOTORLOW;
		osDelay(100);
		pwmVal_motor_target = MOTORMIDTURN;
		while(heading_z<heading_target-DECELERATIONBEARING)
		{
			osDelay(10);
		}
		pwmVal_motor_target = MOTORLOW;
		while(heading_z<heading_target-TURNOVERSHOOT)
				osDelay(10);
		right_dir = 0;
		left_dir = 0;

		osDelay(50);

		target_pwmVal_servo = SERVOCENTER;

		osDelay(150);
	}

	void FrontLeft(int angle)
	{
		move_dir = 'L';
		motor_pid = 0;
		heading_pid = 0;

		target_pwmVal_servo = SERVOLEFT;
		heading_target = heading_target-angle;
		osDelay(100);
		right_dir = 1;
		left_dir = 1;
		pwmVal_motor_target = MOTORLOW;
		osDelay(100);
		pwmVal_motor_target = MOTORMIDTURN;
		while(heading_z>heading_target+DECELERATIONBEARING)
		{
			osDelay(10);
		}
		pwmVal_motor_target = MOTORLOW;
		while(heading_z>heading_target+TURNOVERSHOOT)
					osDelay(10);
		right_dir = 0;
		left_dir = 0;

		osDelay(50);

		target_pwmVal_servo = SERVOCENTER;

		osDelay(150);
	}


	void BackCenter(int dist)
	{
		int cur_dist=0;
		move_dir = 'C';
		target_pwmVal_servo = SERVOCENTER;
		reset_Encoders();
		reset_encoder_pid_error();
		reset_heading_pid_error();
		motor_pid = 1;
		heading_pid = 1;

		left_dist = 0;
		right_dist = 0;
		dist_target = -dist;

		//acceleration start
		right_dir = -1;
		left_dir = -1;
		pwmVal_motor_target = MOTORLOW;
		if(dist_target>-(DECELERATIONDIST+2) && dist_target<-5)//manual acceleration
		{//fix later
			osDelay(50);
			pwmVal_motor_target = MOTORMID;
			while(cur_dist<dist_target-8)
			{
				osDelay(20);
				cur_dist = (left_dist+right_dist)/2;
			}

		}
		else
		{
			//acceleration start
			while(pwmVal_motor_target<MOTORHIGH && cur_dist>dist_target+DECELERATIONDIST)
			{
				pwmVal_motor_target += DECELERATIONRATE;
				cur_dist = (left_dist+right_dist)/2;
				osDelay(20);
			}
			//acceleration end
			while(cur_dist>dist_target+DECELERATIONDIST)
			{
				pwmVal_motor_target = MOTORHIGH;
				osDelay(20);
				cur_dist = (left_dist+right_dist)/2;
			}
		}

		//Deceleration start
		while(pwmVal_motor_target>MOTORLOW && cur_dist>dist_target+MOVEOVERSHOOT)
		{
			pwmVal_motor_target -= DECELERATIONRATE;
			cur_dist = (left_dist+right_dist)/2;
			osDelay(20);
		}
		//Deceleration end
		while(cur_dist>dist_target+MOVEOVERSHOOT)
		{
			pwmVal_motor_target = MOTORLOW;
			cur_dist = (left_dist+right_dist)/2;
			osDelay(10);
		}

		motor_pid = 0;
		heading_pid = 0;

		right_dir = 0;
		left_dir = 0;
		pwmVal_motor_target = 0;

		target_pwmVal_servo = SERVOCENTER;

//		osDelay(50);
	}

	void BackRight(int angle)
	{
		move_dir = 'R';
		motor_pid = 0;
		heading_pid = 0;

		target_pwmVal_servo = SERVORIGHT;
		heading_target = heading_target-angle;
		osDelay(200);
		right_dir = -1;
		left_dir = -1;
		pwmVal_motor_target = MOTORLOW;
		osDelay(100);
		pwmVal_motor_target = MOTORMIDTURN;
		while(heading_z>heading_target+DECELERATIONBEARING)
		{
			osDelay(10);
		}
		pwmVal_motor_target = MOTORLOW;
		while(heading_z>heading_target+TURNOVERSHOOT)
					osDelay(10);
		right_dir = 0;
		left_dir = 0;

		osDelay(50);

		target_pwmVal_servo = SERVOCENTER;

		osDelay(150);
	}

	void BackLeft(int angle)
	{
		move_dir = 'L';
		motor_pid = 0;
		heading_pid = 0;

		target_pwmVal_servo = SERVOLEFT;
		heading_target = heading_target+angle;
		osDelay(200);
		right_dir = -1;
		left_dir = -1;
		pwmVal_motor_target = MOTORLOW;
		osDelay(100);
		pwmVal_motor_target = MOTORMIDTURN;
		while(heading_z<heading_target-DECELERATIONBEARING)
		{
			osDelay(10);
		}
		pwmVal_motor_target = MOTORLOW;
		while(heading_z<heading_target-TURNOVERSHOOT)
					osDelay(10);
		right_dir = 0;
		left_dir = 0;

		osDelay(50);

		target_pwmVal_servo = SERVOCENTER;

		osDelay(150);
	}

	int FrontUltrasound(int dist_target) //dist_target is the distance relative to center wheel
	{
		usDist = dist_target+100;
		int cur_dist=0;
		move_dir = 'C';
		target_pwmVal_servo = SERVOCENTER;
		reset_Encoders();
		reset_encoder_pid_error();
		reset_heading_pid_error();
		motor_pid = 1;
		heading_pid = 1;

		left_dist = 0;
		right_dist = 0;
		dist_target -=20;

		//acceleration start
		right_dir = 1;
		left_dir = 1;
		pwmVal_motor_target = MOTORLOW;

		do
		{
			pwmVal_motor_target += DECELERATIONRATE*USACCERLERATIONMULTIPLIER;
			TriggerUS();
			osDelay(USMINDELAY);
		}while(pwmVal_motor_target<MOTORHIGH && usDist>dist_target+DECELERATIONDIST+16);
		//acceleration end

		if(usDist>dist_target+20 && usDist<dist_target+DECELERATIONDIST+16)//when too near, lower acceleration
		{
			pwmVal_motor_target = MOTORMID;
			while(usDist>dist_target+20)
			{
				TriggerUS();
				osDelay(USMINDELAY);
			}

		}

		while(usDist>dist_target+DECELERATIONDIST+18)
		{
			pwmVal_motor_target = MOTORHIGH;
			TriggerUS();
			osDelay(USMINDELAY);
		}

		while(usDist>dist_target+DECELERATIONDIST+18)
		{
			pwmVal_motor_target = MOTORHIGH;
			TriggerUS();
			osDelay(USMINDELAY);
		}

		//Deceleration start
		do
		{
			pwmVal_motor_target -= DECELERATIONRATE*USACCERLERATIONMULTIPLIER;
			TriggerUS();
			osDelay(USMINDELAY);
		}while(pwmVal_motor_target>MOTORLOW && usDist>dist_target+MOVEUSOVERSHOOT);
		//Deceleration end

		do
		{
			pwmVal_motor_target = MOTORLOW;
			cur_dist = (left_dist+right_dist)/2;
			TriggerUS();
			osDelay(USMINDELAY);
		}while(usDist>dist_target+MOVEUSOVERSHOOT);

		motor_pid = 0;
		heading_pid = 0;

		right_dir = 0;
		left_dir = 0;
		pwmVal_motor_target = 0;

		target_pwmVal_servo = SERVOCENTER;
		cur_dist = (left_dist+right_dist)/2;

		return cur_dist;
	}



	int BackUltrasound(int dist_target) //dist_target is the distance relative to center wheel
	{
		usDist = dist_target-30;
		int cur_dist=0;
		move_dir = 'C';
		target_pwmVal_servo = SERVOCENTER;
		reset_Encoders();
		reset_encoder_pid_error();
		reset_heading_pid_error();
		motor_pid = 1;
		heading_pid = 1;

		left_dist = 0;
		right_dist = 0;
		dist_target -=20;

		//acceleration start
		right_dir = -1;
		left_dir = -1;
		pwmVal_motor_target = MOTORLOW;

		do
		{
			pwmVal_motor_target += DECELERATIONRATE*USACCERLERATIONMULTIPLIER;
			TriggerUS();
			osDelay(USMINDELAY);
		}while(pwmVal_motor_target<MOTORMID && usDist<dist_target-10);
		//acceleration end

		while(usDist<dist_target-10)
		{
			pwmVal_motor_target = MOTORMID;
			TriggerUS();
			osDelay(USMINDELAY);
		}

		//Deceleration start
		do
		{
			pwmVal_motor_target -= DECELERATIONRATE*USACCERLERATIONMULTIPLIER;
			TriggerUS();
			osDelay(USMINDELAY);
		}while(pwmVal_motor_target>MOTORLOW && usDist<dist_target-MOVEUSOVERSHOOT);
		//Deceleration end

		do
		{
			pwmVal_motor_target = MOTORLOW;
			cur_dist = (left_dist+right_dist)/2;
			TriggerUS();
			osDelay(USMINDELAY);
		}while(usDist<dist_target-MOVEUSOVERSHOOT);

		motor_pid = 0;
		heading_pid = 0;

		right_dir = 0;
		left_dir = 0;
		pwmVal_motor_target = 0;

		target_pwmVal_servo = SERVOCENTER;
		cur_dist =(left_dist+right_dist)/2;

		return cur_dist;
	}

	void FrontUltrasoundT2End(int dist_target) //dist_target is the distance relative to center wheel
		{
			usDist = dist_target+40;

			move_dir = 'C';
			target_pwmVal_servo = SERVOCENTER;
			reset_Encoders();
			reset_encoder_pid_error();
			reset_heading_pid_error();
			motor_pid = 1;
			heading_pid = 1;

			left_dist = 0;
			right_dist = 0;
			dist_target -=20;

			//acceleration start
			right_dir = 1;
			left_dir = 1;
			pwmVal_motor_target = MOTORLOW;

			do
			{
				pwmVal_motor_target += DECELERATIONRATE*USACCERLERATIONMULTIPLIER;
				TriggerUS();
				osDelay(USMINDELAY);
			}while(pwmVal_motor_target<MOTORHIGH && usDist>dist_target+15);

			do
			{
				pwmVal_motor_target = MOTORHIGH;
				TriggerUS();
				osDelay(USMINDELAY);
			}while(usDist>dist_target+15);

			//acceleration end

			while(usDist>dist_target)
			{
				pwmVal_motor_target = MOTORLOW;
				TriggerUS();
				osDelay(USMINDELAY);
			}


			motor_pid = 0;
			heading_pid = 0;

			right_dir = 0;
			left_dir = 0;
			pwmVal_motor_target = 0;

			target_pwmVal_servo = SERVOCENTER;
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10, GPIO_PIN_SET);
		}

	int MoveIR_L(int min_dist)//irDistL, min_dist: the minimum movement
	{
		irDistL = DIST_IR_OBSTACLE-1;
		if(min_dist==0){
			for(int i=0; i<10; i++){
				ReadIR();
				osDelay(5);
			}
			if(irDistL>DIST_IR_OBSTACLE){
				return 0;
			}
		}

		int cur_dist=0;
		move_dir = 'C';
		target_pwmVal_servo = SERVOCENTER;
		reset_Encoders();
		reset_encoder_pid_error();
		reset_heading_pid_error();
		motor_pid = 1;
		heading_pid = 1;

		left_dist = 0;
		right_dist = 0;
		right_dir = 1;
		left_dir = 1;
		pwmVal_motor_target = MOTORLOW;
		osDelay(50);
		pwmVal_motor_target = MOTORMIDIR;
		while(cur_dist<min_dist-5)
		{
			osDelay(20);
			cur_dist = (left_dist+right_dist)/2;
		}
		do
		{
			ReadIR();
			osDelay(20);
		}while(irDistL<DIST_IR_OBSTACLE);

		pwmVal_motor_target = MOTORLOW;
		osDelay(100); //Decelerate

		motor_pid = 0;
		heading_pid = 0;

		right_dir = 0;
		left_dir = 0;

		target_pwmVal_servo = SERVOCENTER;
		osDelay(200);
		cur_dist = (left_dist+right_dist)/2 + MOVEIROVERSHOOT;

//		UpdateAllCoordXY(cur_dist);
		return cur_dist;
	}
	int MoveIR_R(int min_dist)
	{
		irDistR = DIST_IR_OBSTACLE-1;
		if(min_dist==0){
			for(int i=0; i<10; i++){
				ReadIR();
				osDelay(5);
			}
			if(irDistR>DIST_IR_OBSTACLE){
				return 0;
			}
		}

		int cur_dist=0;
		move_dir = 'C';
		target_pwmVal_servo = SERVOCENTER;
		reset_Encoders();
		reset_encoder_pid_error();
		reset_heading_pid_error();
		motor_pid = 1;
		heading_pid = 1;

		left_dist = 0;
		right_dist = 0;
		right_dir = 1;
		left_dir = 1;
		pwmVal_motor_target = MOTORLOW;
		osDelay(50);
		pwmVal_motor_target = MOTORMIDIR;
		while(cur_dist<min_dist-5)
		{
			osDelay(20);
			cur_dist = (left_dist+right_dist)/2 + MOVEIROVERSHOOT;
		}
		do
		{
			ReadIR();
			osDelay(20);
		}while(irDistR<DIST_IR_OBSTACLE);

		pwmVal_motor_target = MOTORLOW;
		osDelay(100); //Decelerate

		motor_pid = 0;
		heading_pid = 0;

		right_dir = 0;
		left_dir = 0;

		target_pwmVal_servo = SERVOCENTER;
		osDelay(200);
		cur_dist = (left_dist+right_dist)/2;

//		UpdateAllCoordXY(cur_dist);
		return cur_dist;
	}

	void CalGyroDrift(int timeCal)
	{
		uint32_t millisOld =HAL_GetTick();
		uint32_t millisNow; // time value
		float old_heading_z = heading_z;
		float dt;

		osDelay(timeCal);
		millisNow = HAL_GetTick(); // get the current time
		dt = (millisNow - millisOld)*0.001; // time elapsed in millisecond
		heading_driff += (heading_z-old_heading_z)/dt;
	}

	void ZeroGyro()
	{
		heading_z = 0;
		heading_target = 0;
//		coord_y = 0;
//		coord_x = 10;
//		coord_bearing = 0;
		T2_Dist_y_A = 0;
		T2_Dist_y_B = 0;
		T2_Dist_x_A = 0;
		T2_Dist_x_B = 0;
	}

	void SetCalVar(int calVarVal)
	{
		if(calVarVal == 0)
		{
			indoorOutdoor=0;
			//---INDOOR---
			WHEEL_D_CM = WHEEL_D_CM_INDOOR;
			MOVEOVERSHOOT = MOVEOVERSHOOT_INDOOR;
			TURNOVERSHOOT = TURNOVERSHOOT_INDOOR;

			SERVOLEFT = SERVOLEFT_INDOOR;
			SERVORIGHT = SERVORIGHT_INDOOR;

			MOVEUSOVERSHOOT = MOVEUSOVERSHOOT_INDOOR;
		}
		else
		{
			indoorOutdoor=1;
			WHEEL_D_CM = WHEEL_D_CM_OUTDOOR;
			MOVEOVERSHOOT = MOVEOVERSHOOT_OUTDOOR;
			TURNOVERSHOOT = TURNOVERSHOOT_OUTDOOR;

			SERVOLEFT = SERVOLEFT_OUTDOOR;
			SERVORIGHT = SERVORIGHT_OUTDOOR;

			MOVEUSOVERSHOOT = MOVEUSOVERSHOOT_OUTDOOR;
		}

	}

	//=====TASK 2 MAIN FUNTIONS=====

	//-----T2100-----
	void Task2_100(){

		T2_Dist_y_A = FrontUltrasound(T2FIRSTUSDISTCAMERA);
	}
	//-----T2200-----
	void Task2_200(){
		T2_Dist_y_B = 0;
//		BackCenter(T2FIRSTBACKOFF);
		FrontLeft(60);
		FrontRight(120);
		FrontLeft(60);

		usDist = T2SECONDUSDISTCAMERA;
		for(uint8_t i=0; i<2; i++) {
			TriggerUS();
			osDelay(40);
		}
		if(usDist>T2SECONDUSDISTCAMERA+1){
			T2_Dist_y_B += FrontUltrasound(T2SECONDUSDISTCAMERA);
		}
		else if(usDist<T2SECONDUSDISTCAMERA-1){
			T2_Dist_y_B += BackUltrasound(T2SECONDUSDISTCAMERA);
		}


	}
	//-----T2201-----
	void Task2_201(){
		T2_Dist_y_B = 0;
//		BackCenter(T2FIRSTBACKOFF);
		FrontRight(60);
		FrontLeft(120);
		FrontRight(60);

		usDist = T2SECONDUSDISTCAMERA;
		for(uint8_t i=0; i<2; i++) {
			TriggerUS();
			osDelay(40);
		}
		if(usDist>T2SECONDUSDISTCAMERA+1){
			T2_Dist_y_B += FrontUltrasound(T2SECONDUSDISTCAMERA);
		}
		else if(usDist<T2SECONDUSDISTCAMERA-1){
			T2_Dist_y_B += BackUltrasound(T2SECONDUSDISTCAMERA);
		}


	}

	//-----T2300-----
	void Task2_300(){
		//BackCenter(T2SECONDBACKOFF);

		FrontLeft(90);
		T2_Dist_x_A = MoveIR_R(0);

		FrontRight(180);
		T2_Dist_x_B = MoveIR_R((int) TURNRADIUS);

		FrontRight(90);

		T2_return_y = T2_Dist_y_A + T2_Dist_y_B + 3.4*TURNRADIUS - T2ENTRYDIST - T2ROBOTOFFSET - T2FIRSTBACKOFF - T2SECONDBACKOFF;

		FrontCenter(T2_return_y);

		FrontRight(90);
		T2_return_x = T2_Dist_x_B - T2_Dist_x_A - 2*TURNRADIUS + 5; //may want to remove the +5

		if(T2_return_x>=0)
		{
			FrontCenter(abs(T2_return_x));
		}
		else
		{
			BackCenter(abs(T2_return_x));
		}

		FrontLeft(90);

		FrontUltrasoundT2End(42);

		//FrontUltrasound(40);



	}

	//-----T2301-----
	void Task2_301(){
//		BackCenter(T2SECONDBACKOFF);

		FrontRight(90);
		T2_Dist_x_A = MoveIR_L(0);

		FrontLeft(180);
		T2_Dist_x_B = MoveIR_L((int) TURNRADIUS);

		FrontLeft(90);

		T2_return_y = T2_Dist_y_A + T2_Dist_y_B + 3.4*TURNRADIUS - T2ENTRYDIST - T2ROBOTOFFSET - T2FIRSTBACKOFF - T2SECONDBACKOFF;

		FrontCenter(T2_return_y);

		FrontLeft(90);
		T2_return_x = T2_Dist_x_B - T2_Dist_x_A - 2*TURNRADIUS + 5; //may want to remove the +5


		if(T2_return_x>=0)
		{
			FrontCenter(abs(T2_return_x));
		}
		else
		{
			BackCenter(abs(T2_return_x));
		}

		FrontRight(90);

		FrontUltrasoundT2End(42);

//		FrontUltrasound(40);

	}

	void TestFunc()
	{
//		sprintf(oled_display[5], "usDist:INIT  ");
		while(1)
		{
			ReadIR();
			osDelay(20);
//			sprintf(oled_display[5], "IR L:%d R:%d  ",(int)irDistL, (int)irDistR);

		}
	}

	void TriggerUS()
	{
		//Set TRIG to low for awhile
		HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_RESET);
		delay_us(5);

		//Output 1us of Trig
		HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_SET);
		delay_us(10);
		HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, GPIO_PIN_RESET);
		//wait for rising edge
		HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); //enable timer capture mode
	}

	float irValueToDist(uint16_t value_ir) {
		float div_ir = pow(((float) value_ir) / 4095, 1.226);
		float dist_ir = (div_ir < 6.3028 / DIST_IR_MAX)
			? DIST_IR_MAX
			: 6.3028 / div_ir;

//		dist_ir -= DIST_IR_OFFSET;
		if (dist_ir < DIST_IR_MIN) dist_ir = DIST_IR_MIN;
		return dist_ir;
	}

	void ReadIR()
	{
		HAL_ADC_Start(&hadc1);// left adc
		HAL_ADC_Start(&hadc2);// right adc

		while (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK
				&& HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) != HAL_OK);

		uint16_t lValIR = (uint16_t) HAL_ADC_GetValue(&hadc1);
		uint16_t rValIR = (uint16_t) HAL_ADC_GetValue(&hadc2);
//		sprintf(oled_display[5], "L:%d R:%d ", lValIR, rValIR);

		irDistL = 0.82*irDistL + (1-0.82)*irValueToDist(lValIR); //WITH FILTER
		irDistR = 0.82*irDistR + (1-0.82)*irValueToDist(rValIR); //WITH FILTER
	}


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
		osDelay(10000);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
	/* USER CODE BEGIN motor */
		uint16_t pwmVal = 0;
		int temp_pwmVal_servo=0;
	//	right_target = 0, left_target = 0;
	//	right_dir = 0, left_dir = 0;

		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  /* Infinite loop */
	  for(;;)
	  {
		  left_pwmVal_motor = pwmVal_motor_target;
		  right_pwmVal_motor = pwmVal_motor_target;

		  if (motor_pid == 1){
			  if((int)left_pwmVal_motor - motor_correction > MOTORMAX)
				  left_pwmVal_motor = MOTORMAX;
			  else if ((int)left_pwmVal_motor - motor_correction < MOTORMIN)
				  left_pwmVal_motor = MOTORMIN;
			  else
				  left_pwmVal_motor = left_pwmVal_motor - motor_correction;

			  if((int)right_pwmVal_motor + motor_correction > MOTORMAX)
				  right_pwmVal_motor = MOTORMAX;
			  else if ((int)right_pwmVal_motor + motor_correction < MOTORMIN)
				  right_pwmVal_motor = MOTORMIN;
			  else
				  right_pwmVal_motor = right_pwmVal_motor + motor_correction;
		  }

		  if(move_dir == 'R')
			  right_pwmVal_motor = right_pwmVal_motor*TURNRATIO;
		  if(move_dir == 'L')
			  left_pwmVal_motor = left_pwmVal_motor*TURNRATIO;

		  /* Speed control corrected 2026-08-25: this board's AT8236 VREF pins
		   * are hardwired to 3V3 (see main.h), so there's no separate speed
		   * pin to PWM like the old htim8 CH1/CH3 lines assumed -- speed is
		   * now set by PWM-ing whichever direction channel is active and
		   * holding the other at 0, same left_pwmVal_motor/right_pwmVal_motor
		   * values as before, just retargeted to the real IN1/IN2 pins. */

		  //LEFT MOTOR (PB9=IN1=TIM4_CH4, PB8=IN2=TIM4_CH3)
		  if (left_dir>0)//move forward
		  {
			  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,left_pwmVal_motor);
			  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
		  }
		  else if (left_dir<0)//move backward
		  {
			  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,left_pwmVal_motor);
			  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
		  }
		  else	//left_dir==0 dont move
		  {
			  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
			  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
		  }

		  //RIGHT MOTOR (PE14=IN1=TIM1_CH4, PE13=IN2=TIM1_CH3, MOTORD/U12)
		  if (right_dir>0)//move forward
		  {
			  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,right_pwmVal_motor);
			  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		  }
		  else if (right_dir<0)//move backward
		  {
			  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,right_pwmVal_motor);
			  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
		  }
		  else	//right_dir==0 dont move
		  {
			  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
			  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
		  }

		  // SERVO
		  temp_pwmVal_servo = target_pwmVal_servo+heading_correction;
	//	  sprintf(oled_display[2], "T:%d\0  ", temp_pwmVal_servo);
		  if (heading_pid == 1){
			  if(temp_pwmVal_servo>SERVOMAX)
			  {
				  temp_pwmVal_servo = SERVOMAX;
			  }
			  else if(temp_pwmVal_servo<SERVOMIN)
			  {
				  temp_pwmVal_servo = SERVOMIN;
			  }
			  pwmVal_servo = temp_pwmVal_servo;
		  }
		  else
		  {
			  pwmVal_servo = target_pwmVal_servo;
		  }

	//	  sprintf(oled_display[5], "S pwm:%d\0   ", pwmVal_servo);
		   htim8.Instance->CCR1 = pwmVal_servo;	//extreme right
	   osDelay(10); //30
	  }
  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_encoder_task */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_encoder_task */
void encoder_task(void *argument)
{
  /* USER CODE BEGIN encoder_task */
  /* Infinite loop */
	uint8_t hello[20];

	uint32_t millisOld =HAL_GetTick();
	uint32_t millisNow; // time value
	motor_correction=0;


	float dt;// time elapse

  for(;;)
  {
	  left_dist = (get_left_encoder()/MOTOR_PPR)*WHEEL_D_CM*PI;
	  right_dist = (get_right_encoder()/MOTOR_PPR)*WHEEL_D_CM*PI;

	  //UPDATE MOTOR ERROR SUTFF HERE FOR PID!!!!!
	  motor_correction = motor_pid_correction(left_dist, right_dist, left_dir, right_dir);
//	  sprintf(oled_display[5], "corr:%d   ", motor_correction);

	  sprintf(oled_display[1], "L:%.2f\0   ", left_dist);
	  sprintf(oled_display[2], "R:%.2f\0   ", right_dist);

//	  sprintf(oled_display[1], "L:%d    ", get_left_encoder());
//	  sprintf(oled_display[2], "R:%d    ", get_right_encoder());


	  osDelay(10); //30
  }
  /* USER CODE END encoder_task */
}

/* USER CODE BEGIN Header_comm_task */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comm_task */
void comm_task(void *argument)
{
  /* USER CODE BEGIN comm_task */
  /* Infinite loop */
	//	right_target = 0, left_target = 0;
	//	right_dir = 0, left_dir = 0;
	uint8_t hello[20];
	uint8_t cur_inst[5];
	int instr_value=0;	//value of instruction to be executed

	sprintf(oled_display[0], "Task:  ",instr_value);

  for(;;)
  {
	if(receivedInstruction == 1){
		for(uint8_t instrIndex=0; instrIndex<instrLen-1; instrIndex++){
			memcpy(cur_inst, instrList[instrIndex], sizeof(instrList[instrIndex]));
//			sprintf(oled_display[3], "Rx:%s        ", cur_inst);
			instr_value = atoi(&cur_inst[2]);

			switch((((uint16_t)cur_inst[0]) << 8 ) | (cur_inst[1] & 0xFF))
			{
				case 'TO':	//TIME OUT 10 SECOND
//					reset_heading_pid_error();
//					heading_pid = 1;
					sprintf(oled_display[0], "Task: TO%d  ",instr_value);
					osDelay(10000);
//					heading_pid = 0;
					break;

					break;
				case 'FR':	//FrontRight
					sprintf(oled_display[0], "Task: FR%d  ",instr_value);
					FrontRight(instr_value);
					break;

				case 'FL':	//FrontLeft
					sprintf(oled_display[0], "Task: FL%d  ",instr_value);
					FrontLeft(instr_value);
					break;

				case 'FC':	//FrontCenter
					sprintf(oled_display[0], "Task: FC%d  ",instr_value);
					FrontCenter(instr_value);
					break;

				case 'BR':	//BackRight
					sprintf(oled_display[0], "Task: BR%d  ",instr_value);
					BackRight(instr_value);
					break;


				case 'BL':	//BackLeft
					sprintf(oled_display[0], "Task: BL%d  ",instr_value);
					BackLeft(instr_value);
					break;

				case 'BC':	//BackCenter
					sprintf(oled_display[0], "Task: BC%d  ",instr_value);
					BackCenter(instr_value);
					break;

				case 'GC':	//CalGyroDrift
					sprintf(oled_display[0], "Task: GC     ");
					SetCalVar(instr_value);
					CalGyroDrift(4000);
					ZeroGyro();
					break;

				case 'G0':
					sprintf(oled_display[0], "Task: G0     ");
					ZeroGyro();
					break;

				case 'FU':	//Move forward until specified ultrasound distance relative to center of wheel
					sprintf(oled_display[0], "Task: FU     ");
					FrontUltrasound(instr_value);
					break;

				case 'BU':	//Move Backward until specified ultrasound distance relative to center of wheel
					sprintf(oled_display[0], "Task: FU     ");
					BackUltrasound(instr_value);
					break;



				case 'T2': // TASK 2
					switch(instr_value)
					{
						case 100:
							sprintf(oled_display[0], "Task: T2100  ");
							Task2_100();
							break;

						case 200:
							sprintf(oled_display[0], "Task: T2200  ");
							Task2_200();
							break;

						case 201:
							sprintf(oled_display[0], "Task: T2201  ");
							Task2_201();
							break;

						case 300:
							sprintf(oled_display[0], "Task: T2300  ");
							Task2_300();
							break;

						case 301:
							sprintf(oled_display[0], "Task: T2301  ");
							Task2_301();
							break;

						default:
							break;
					}
					break;

				case 'IR':
					if(instr_value==0)
					{
						sprintf(oled_display[0], "Task: IR000  ");
						MoveIR_L(0);
					}
					else if(instr_value==1)
					{
						sprintf(oled_display[0], "Task: IR001  ");
						MoveIR_R(0);

					}
					break;


				case 'TF':	// Test
					sprintf(oled_display[0], "Task: TF     ");
//					osDelay(1000);
					TestFunc();
					break;

				default:
					//OLED_ShowString(0, 0, "Task: ERROR");
					break;
			}
//			SendCoordUart();
			osDelay(10);
		}
		sprintf(oled_display[0], "Task: Nil   ");
		osDelay(200);
		HAL_UART_Transmit(&huart3, (uint8_t *) "FIN\r\n", 5, 0xFFFF);
		receivedInstruction=0;
		instrLen = 0;
	}
	osDelay(20);
  }
//	osDelay(10);
//  HAL_UART_Transmit(&huart3, (uint8_t *) &ch, 1, 0xFFFF);
//  if(ch<'Z')
//	  ch++;
//  else
//	  ch = 'A';
//  osDelay(5000);

//	  while(1)
//		  osDelay(10);
  /* USER CODE END comm_task */
}

/* USER CODE BEGIN Header_gyro_task */
/**
* @brief Function implementing the GyroTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyro_task */
void gyro_task(void *argument)
{
  /* USER CODE BEGIN gyro_task */
  /* Infinite loop */
	uint8_t hello[20];
	uint32_t millisOld =HAL_GetTick();
	uint32_t millisNow; // time value

	float dt;// time elapse
	heading_z = 0;
	heading_driff = 0;
	for(;;)
	{
		millisNow = HAL_GetTick(); // get the current time
		dt = (millisNow - millisOld)*0.001; // time elapsed in millisecond
		millisOld = millisNow;

		sensors_read_gyroZ();

		heading_z = heading_z + sensors.gyroZ*dt-heading_driff*dt;

		heading_correction = heading_pid_correction(heading_target, heading_z, left_dir, right_dir);
//		sprintf(oled_display[5], "corr:%d   ", heading_correction);
//		while (heading_z >= 360.0f) {
//		    heading_z -= 360.0f;
//		}
//		while (heading_z < 0.0f) {
//		    heading_z += 360.0f;
//		}
		// GYRO DISPLAY TEST
		if(heading_z>=0)
		  sprintf(oled_display[4], "Gyro: +%d.%02d", abs((int)heading_z), abs((int)((heading_z - (int)heading_z) * 100)));
		else
		  sprintf(oled_display[4], "Gyro: -%d.%02d", abs((int)heading_z), abs((int)((heading_z - (int)heading_z) * 100)));


		osDelay(10); //20
	}
  /* USER CODE END gyro_task */
}

/* USER CODE BEGIN Header_oled_update_task */
/**
* @brief Function implementing the oledUpdateTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oled_update_task */
void oled_update_task(void *argument)
{
  /* USER CODE BEGIN oled_update_task */
	int tiktok = 1;
  /* Infinite loop */
  for(;;)
  {
//	  tiktok = -tiktok;
//	  sprintf(oled_display[5], "Tik tok:%d  ",tiktok);
//	  sprintf(oled_display[5], "B:%d X:%d Y:%d  ",coord_bearing, coord_x, coord_y);	//direction
//	  sprintf(oled_display[5], "IndoorOutdoor:%d",indoorOutdoor);

	  int8_t coord_bearing = 0; //0:N E,S,W
	  OLED_ShowString(0, 0, oled_display[0]);
	  OLED_ShowString(0, 10, oled_display[1]);
	  OLED_ShowString(0, 20, oled_display[2]);
	  OLED_ShowString(0, 30, oled_display[3]);
	  OLED_ShowString(0, 40, oled_display[4]);
	  OLED_ShowString(0, 50, oled_display[5]);
	  OLED_Refresh_Gram();
	  osDelay(400);
  }
  /* USER CODE END oled_update_task */
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
