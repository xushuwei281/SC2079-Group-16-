/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
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
#include "ICM20948.h"
#include <stdio.h>
#include <string.h>
#include "pidHeading.h"
#include "pidMotor.h"
#include <math.h>
#include <stdlib.h>    /* atoi, abs */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for servo
#define PWM_PERIOD  1600
#define SERVOCENTER 146      /* was ~71.5 when tim8 prescale is 320 */
#define SERVOLEFT   101
#define SERVORIGHT  215
#define SERVOMIN     94
#define SERVOMAX    231
//testing servo center:
#define DRIVE_MS      4000
#define DRIVE_SPEED   480
#define DRIVE_SLOW    280       /* creep speed for the last few cm      */
/* speeds - reference values x 1600/7200 */
#define MOTORLOW    280     /* increased for reliable starting torque and turn drive */
#define MOTORMID    580     /* cruise speed */
#define MOTORHIGH   890     /* 4000 -> 889                              */
#define MOTORMIN     150
#define MOTORMAX     1330
#define TARGET_CM     120.0f    /* supervisor's number for A.3          */
#define MOVEOVERSHOOT  1.9f    /* coast after cutting power - MEASURE  */
#define RAMP_CM        15.0f    /* start creeping this far from target  */

//turn
#define TURNRATIO      0.80f   /* inner wheel powered adequately to prevent stall */
#define TURNOVERSHOOT  1.0f    /* deg it keeps turning after stop - MEASURE */

//for UART communication
#define FRAME_LEN 5

static volatile uint8_t rxBuffer[2][FRAME_LEN];   /* double-buffered per their §6.1 */
static volatile uint8_t rxIdx = 0;

volatile uint8_t instrList[40][FRAME_LEN];
volatile uint8_t instrLen = 0;
volatile uint8_t runRequested = 0;      /* their "receivedInstruction" */
volatile uint8_t estopFlag = 0;
volatile uint32_t rxCount = 0;

static float batchDist = 0.0f;          /* cm accumulated this batch - see D */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal3,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for GyroTask */
osThreadId_t GyroTaskHandle;
const osThreadAttr_t GyroTask_attributes = {
  .name = "GyroTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for Comm_task */
osThreadId_t Comm_taskHandle;
const osThreadAttr_t Comm_task_attributes = {
  .name = "Comm_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* USER CODE BEGIN PV */
float gyroZ = 0.0f;
float gyroOffset = 0.0f;
float correctedZ = 0.0f;
//for heading PID
float headingTarget     = 0.0f;   /* where we want to point   */
int   headingCorrection = 0;      /* servo counts to add      */
uint8_t heading_pid     = 0;      /* 1 = PID on, 0 = off      */
float angleMaxDev = 0.0f;
float angleNow = 0.0f;
uint32_t gyroLastTick = 0;
char gyroMsg[32];
float    gyroDrift    = 0.0f;    /* residual walk, deg/s */
uint32_t gyroBadReads = 0;       /* diagnostic counter   */
char     oled_display[6][16];    /* shared display buffer */
//for motor
int     motorCorrection = 0;
uint8_t motor_pid = 0;

//Motor
int32_t  dist_target = 0;
float    left_dist = 0, right_dist = 0;
int8_t   left_dir = 0, right_dir = 0;   /* -3..3 : sign=direction, magnitude=speed tier; 1 low, 3 high */
uint16_t left_pwmVal_motor = MOTORMID, right_pwmVal_motor = MOTORMID;
int8_t   move_dir = 'C';                /* C:Center  L:Left  R:Right */

//Servo
uint16_t target_pwmVal_servo = SERVOCENTER;
uint16_t pwmVal_servo        = SERVOCENTER;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument);
void encoder_task(void *argument);
void motor(void *argument);
void gyro_task(void *argument);
void comm_task(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Encoder
static int32_t encoderA(void) { return (int32_t)__HAL_TIM_GET_COUNTER(&htim2); }
static int32_t encoderB(void) { return -(int16_t)__HAL_TIM_GET_COUNTER(&htim3); }
//motor
#define MOTOR_PPR     	1527.0f	//1320.0f
#define WHEEL_D_CM       6.5f
#define CM_PER_COUNT  (WHEEL_D_CM * 3.1415f / MOTOR_PPR)
static float distA(void) { return encoderA() * CM_PER_COUNT; }
static float distB(void) { return encoderB() * CM_PER_COUNT; }
static void encodersZero(void)
{
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   __HAL_TIM_SET_COUNTER(&htim3, 0);
}
static void setMotorA(int16_t speed)
{
   if (speed >  (PWM_PERIOD - 1)) speed =  (PWM_PERIOD - 1);
   if (speed < -(PWM_PERIOD - 1)) speed = -(PWM_PERIOD - 1);
   if (speed > 0) {
       __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);       /* IN1 held high */
       __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);   /* IN2 pulsed    */
   } else if (speed < 0) {
       __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);       /* IN2 held high */
       __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -speed);  /* IN1 pulsed    */
   } else {
       __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);       /* both high     */
       __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);       /* = brake       */
   }
}
static void setMotorB(int16_t speed)
{
   if (speed >  (PWM_PERIOD - 1)) speed =  (PWM_PERIOD - 1);
   if (speed < -(PWM_PERIOD - 1)) speed = -(PWM_PERIOD - 1);
   if (speed > 0) {
       __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);       /* IN2 PE6 high  */
       __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);   /* IN1 PE5 pulsed*/
   } else if (speed < 0) {
       __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
       __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -speed);
   } else {
       __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
       __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
   }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t sbuf[15] = "Hello World!\n\r";
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
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
 OLED_Init();
 ICM20948_init(&hi2c2,
               0,
               GYRO_FULL_SCALE_250DPS,
               ACCEL_FULL_SCALE_2G);
 /* Gyroscope calibration - keep robot stationary */
 gyroOffset = 0.0f;
 for (int i = 0; i < 200; i++)
 {
     ICM20948_readGyroscope_Z(
         &hi2c2,
         0,
         GYRO_FULL_SCALE_250DPS,
         &gyroZ
     );
     gyroOffset += gyroZ;
     HAL_Delay(5);
 }
 gyroOffset /= 200.0f;
 /* --- Residual drift calibration ---------------------------------------
    Removing the offset is only the first layer. What is left still makes
    the integrated angle walk, so measure that walk over 4 s and subtract
    it every cycle. The board must stay completely still through this. */
 OLED_ShowString(0,  0, (uint8_t *)"Calibrating...");
 OLED_ShowString(0, 20, (uint8_t *)"Keep robot still");
 OLED_Refresh_Gram();
 angleNow     = 0.0f;
 gyroDrift    = 0.0f;
 gyroLastTick = HAL_GetTick();
 uint32_t driftStart = gyroLastTick;
 while ((HAL_GetTick() - driftStart) < 4000)
 {
     ICM20948_readGyroscope_Z(
         &hi2c2,
         0,
         GYRO_FULL_SCALE_250DPS,
         &gyroZ
     );
     uint32_t now = HAL_GetTick();
     angleNow += (gyroZ - gyroOffset) * ((now - gyroLastTick) * 0.001f);
     gyroLastTick = now;
     HAL_Delay(10);
 }
 gyroDrift = angleNow / 4.0f;   /* degrees of walk per second */
 angleNow  = 0.0f;
 /* Seed the display buffer that StartDefaultTask paints from.
    Off: is the zero-rate bias in deg/s. If it reads small (under ~20) the
    250 dps range is fine; if it is large, the range is too narrow and
    something else needs looking at.
    Drf: is the residual drift in HUNDREDTHS of a deg/s. */
 snprintf(oled_display[0], sizeof(oled_display[0]), "SC2079 MDP G16 ");
 snprintf(oled_display[1], sizeof(oled_display[1]), "Off:%-10d", (int)(gyroOffset * 100.0f));
 snprintf(oled_display[4], sizeof(oled_display[4]), "Drf:%-10d", (int)(gyroDrift * 100.0f));
 snprintf(oled_display[5], sizeof(oled_display[5]), "%-15s", "ready");
//
//  OLED_ShowString(10, 5, (uint8_t *)"SC2079/MDP_Hi");
//
//  OLED_buf = (uint8_t *)"MDP_Grp16";
//  OLED_ShowString(40, 30, OLED_buf);
 OLED_Refresh_Gram();
 //for UART receive
 HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer[0], FRAME_LEN);
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

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(encoder_task, NULL, &EncoderTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(motor, NULL, &MotorTask_attributes);

  /* creation of GyroTask */
  GyroTaskHandle = osThreadNew(gyro_task, NULL, &GyroTask_attributes);

  /* creation of Comm_task */
  Comm_taskHandle = osThreadNew(comm_task, NULL, &Comm_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
 /* add threads, ... */
 if (GyroTaskHandle == NULL) {
     snprintf(oled_display[5], sizeof(oled_display[5]), "%-15s", "GYRO TASK FAIL");
 }
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
	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);  // ON  (active-low)
	  HAL_Delay(2000);
	  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);    // OFF
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
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */

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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1599;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
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
  htim8.Init.Prescaler = 160;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */
  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */
  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1599;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DC_Pin|OLED_REST_Pin|OLED_SDIN_Pin|OLED_SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin OLED_REST_Pin OLED_SDIN_Pin OLED_SCLK_Pin */
  GPIO_InitStruct.Pin = DC_Pin|OLED_REST_Pin|OLED_SDIN_Pin|OLED_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//for UART communication
///* Send a 3-letter reply to the RPi and mirror it to the OLED. */
//static void uartSend(const char *code)
//{
//    char frame[8];
//    int n = snprintf(frame, sizeof(frame), "%s\r\n", code);
//    HAL_UART_Transmit(&huart3, (uint8_t *)frame, n, 10);
//    snprintf(oled_display[1], sizeof(oled_display[1]), "Tx:%-12s", code);
//}

static void MotorsOff(void)
{
    /* CCR 0 on both channels = both driver pins HIGH = brake (polarity LOW) */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    htim8.Instance->CCR2 = SERVOCENTER;

    /* CRITICAL: otherwise motor() re-drives them 40 ms later */
    left_dir = 0;  right_dir = 0;
    motor_pid = 0; heading_pid = 0;
}

static void EStop(void)
{
    estopFlag = 1;
    MotorsOff();
    snprintf(oled_display[0], sizeof(oled_display[0]), "E STOP         ");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance != USART3) return;

   uint8_t *pkt = (uint8_t *)rxBuffer[rxIdx];
   rxIdx ^= 1;
   HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer[rxIdx], FRAME_LEN);

   rxCount++;
   if (estopFlag) return;                       /* latched until reboot */

   if (pkt[0] == 'Q') {
       EStop();
   }
   else if (pkt[0] == '#') {
       runRequested = 1;
       HAL_UART_Transmit(&huart3, (uint8_t *)"RUN\r\n", 5, 10);
   }
   else if (instrLen >= 40) {
       HAL_UART_Transmit(&huart3, (uint8_t *)"FUL\r\n", 5, 10);
   }
   else if (runRequested == 1) {
       HAL_UART_Transmit(&huart3, (uint8_t *)"BUS\r\n", 5, 10);
   }
   else {
       memcpy((void *)instrList[instrLen++], pkt, FRAME_LEN);
       /* no ACK - the RPi protocol does not expect one */
   }

   snprintf(oled_display[0], sizeof(oled_display[0]), "Rx:%.5s       ", (char *)pkt);
   snprintf(oled_display[3], sizeof(oled_display[3]), "N%-3d R%d E%d C%-4lu",
            (int)instrLen, (int)runRequested, (int)estopFlag,
            (unsigned long)rxCount);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART3) {
       __HAL_UART_CLEAR_OREFLAG(huart);
       __HAL_UART_CLEAR_NEFLAG(huart);
       __HAL_UART_CLEAR_FEFLAG(huart);
       __HAL_UART_CLEAR_PEFLAG(huart);
       HAL_UART_Receive_IT(&huart3, (uint8_t *)rxBuffer[rxIdx], FRAME_LEN);
   }
}

/* ---------------- movement primitives (his structure) ---------------- */

void FrontCenter(int dist)
{
    int cur_dist = 0;
    move_dir = 'C';
    target_pwmVal_servo = SERVOCENTER;
    encodersZero();
    reset_motor_pid_error();
    reset_heading_pid_error();
    headingTarget = angleNow; /* Lock current heading for straight driving */
    motor_pid   = 1;
    heading_pid = 1;

    left_dist = 0;  right_dist = 0;
    dist_target = dist;

    right_dir = 1;  left_dir = 1;      /* MOTORLOW - break static friction */
    osDelay(200);
    right_dir = 2;  left_dir = 2;      /* MOTORMID - cruise */

    while (cur_dist < dist_target - 5) {
        osDelay(20);
        cur_dist = (int)((left_dist + right_dist) / 2.0f);
    }

    right_dir = 1;  left_dir = 1;      /* creep the last 5 cm */
    while (((left_dist + right_dist) / 2.0f) < dist_target - MOVEOVERSHOOT)
        osDelay(10);

    motor_pid = 0;  heading_pid = 0;
    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    batchDist += 0.5f * (left_dist + right_dist);
    osDelay(100);
}

void BackCenter(int dist)
{
    int cur_dist = 0;
    move_dir = 'C';
    target_pwmVal_servo = SERVOCENTER;
    encodersZero();
    reset_motor_pid_error();
    reset_heading_pid_error();
    headingTarget = angleNow; /* Lock current heading for straight reversing */
    motor_pid   = 1;
    heading_pid = 1;

    left_dist = 0;  right_dist = 0;
    dist_target = -dist;

    right_dir = -1;  left_dir = -1;
    osDelay(200);
    right_dir = -2;  left_dir = -2;

    while (cur_dist > dist_target + 5) {
        osDelay(20);
        cur_dist = (int)((left_dist + right_dist) / 2.0f);
    }

    right_dir = -1;  left_dir = -1;
    while (((left_dist + right_dist) / 2.0f) > dist_target + MOVEOVERSHOOT)
        osDelay(10);

    motor_pid = 0;  heading_pid = 0;
    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    batchDist += 0.5f * (left_dist + right_dist);
    osDelay(100);
}

void FrontRight(int angle)
{
    move_dir = 'R';
    motor_pid = 0;  heading_pid = 0;        /* PIDs OFF for the whole turn */

    target_pwmVal_servo = SERVORIGHT;
    osDelay(200);                            /* let the servo physically arrive */

    float startAngle = angleNow;
    right_dir = 2;  left_dir = 2;            /* cruise speed */

    while (fabsf(angleNow - startAngle) < (angle - 4))  osDelay(15);

    right_dir = 1;  left_dir = 1;            /* creep the last 4 deg */
    while (fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT))  osDelay(10);

    right_dir = 0;  left_dir = 0;
    osDelay(100);
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(100);
}

void FrontLeft(int angle)
{
    move_dir = 'L';
    motor_pid = 0;  heading_pid = 0;

    target_pwmVal_servo = SERVOLEFT;
    osDelay(200);

    float startAngle = angleNow;
    right_dir = 2;  left_dir = 2;

    while (fabsf(angleNow - startAngle) < (angle - 4))  osDelay(15);

    right_dir = 1;  left_dir = 1;
    while (fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT))  osDelay(10);

    right_dir = 0;  left_dir = 0;
    osDelay(100);
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(100);
}

void BackRight(int angle)
{
    move_dir = 'R';
    motor_pid = 0;  heading_pid = 0;

    target_pwmVal_servo = SERVORIGHT;
    osDelay(200);

    float startAngle = angleNow;
    right_dir = -2;  left_dir = -2;

    while (fabsf(angleNow - startAngle) < (angle - 4))  osDelay(15);

    right_dir = -1;  left_dir = -1;
    while (fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT))  osDelay(10);

    right_dir = 0;  left_dir = 0;
    osDelay(100);
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(100);
}

void BackLeft(int angle)
{
    move_dir = 'L';
    motor_pid = 0;  heading_pid = 0;

    target_pwmVal_servo = SERVOLEFT;
    osDelay(200);

    float startAngle = angleNow;
    right_dir = -2;  left_dir = -2;

    while (fabsf(angleNow - startAngle) < (angle - 4))  osDelay(15);

    right_dir = -1;  left_dir = -1;
    while (fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT))  osDelay(10);

    right_dir = 0;  left_dir = 0;
    osDelay(100);
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(100);
}

void CalGyroDrift(int timeCal)
{
    float    old = angleNow;
    uint32_t t0  = HAL_GetTick();
    osDelay(timeCal);
    float dt = (HAL_GetTick() - t0) * 0.001f;
    gyroDrift += (angleNow - old) / dt;      /* += refines on repeat calls */
}

void ZeroGyro(void)
{
    angleNow      = 0.0f;
    headingTarget = 0.0f;
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
 /* This task owns the OLED. No other task may call OLED_* - they write into
    oled_display[] instead. A refresh is a ~90 ms bit-banged SPI burst of 1024
    bytes, so it must never sit inside a control loop. */
 for(;;)
 {
   OLED_ShowString(0,  0, (uint8_t *)oled_display[0]);
   OLED_ShowString(0, 10, (uint8_t *)oled_display[1]);
   OLED_ShowString(0, 20, (uint8_t *)oled_display[2]);
   OLED_ShowString(0, 30, (uint8_t *)oled_display[3]);
   OLED_ShowString(0, 40, (uint8_t *)oled_display[4]);
   OLED_ShowString(0, 50, (uint8_t *)oled_display[5]);
   OLED_Refresh_Gram();
   osDelay(400);
 }
  /* USER CODE END 5 */
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
		  for(;;)
		  {
		      left_dist  = distA();
		      right_dist = distB();

		      motorCorrection = motor_pid_correction(left_dist, right_dist);

		      snprintf(oled_display[1], sizeof(oled_display[1]),
		               "L:%-5d R:%-5d", (int)left_dist, (int)right_dist);

		      osDelay(10);
		  }
	  /* USER CODE END encoder_task */
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
//
	  // Start Motor A PWM
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	  // Start Motor B PWM
	  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	  // Servo PWM
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  htim8.Instance->CCR2 = SERVOCENTER;
	  // Encoders (TIM2: PA15+PB3 = Motor A,  TIM3: PB4+PB5 = Motor B)
	  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	  // Stop both motors first
	  setMotorA(0);
	  setMotorB(0);
	  osDelay(500);
//
//	  // ---- ONE straight run, then park. Press RESET for another. ----
//
//	  htim8.Instance->CCR2 = SERVOCENTER;
//	  osDelay(3000);                     /* line the car up and let go */
//
//	  reset_heading_pid_error();
//	  headingTarget = angleNow;
//	  angleMaxDev   = 0.0f;
//	  heading_pid   = 0;                 /* PID OFF - encoders are the test now */
//	  encodersZero();
//	  reset_motor_pid_error();
//	  motor_pid = 0;
//
//
//	  snprintf(oled_display[4], sizeof(oled_display[4]), "GO   C:%-5d", SERVOCENTER);
//	  for (int t = 0; t < DRIVE_MS; t += 10)
//	  {
//	    /* ---- motors: re-apply the correction every tick ---- */
//	    int spA = DRIVE_SPEED, spB = DRIVE_SPEED;
//	    if (motor_pid) {
//	        spA = DRIVE_SPEED - motorCorrection;
//	        spB = DRIVE_SPEED + motorCorrection;
//	        if (spA > MOTOR_MAX) spA = MOTOR_MAX;
//	        if (spA < MOTOR_MIN) spA = MOTOR_MIN;
//	        if (spB > MOTOR_MAX) spB = MOTOR_MAX;
//	        if (spB < MOTOR_MIN) spB = MOTOR_MIN;
//	    }
//	    setMotorA(spA);
//	    setMotorB(spB);
//
//	    /* ---- servo ---- */
//	    int servoVal = SERVOCENTER + headingCorrection;
//	    if (servoVal > SERVOMAX) servoVal = SERVOMAX;
//	    if (servoVal < SERVOMIN) servoVal = SERVOMIN;
//	    if (!heading_pid) servoVal = SERVOCENTER;
//	    htim8.Instance->CCR2 = servoVal;
//
//	    osDelay(10);
//	  }
//
//	  heading_pid = 0;
//	  htim8.Instance->CCR2 = SERVOCENTER;
//	  setMotorA(0);
//	  setMotorB(0);
//	  snprintf(oled_display[4], sizeof(oled_display[4]), "%-15s", "DONE - measure");
//
//	  /* park here forever, motors held stopped */
//	  for(;;)
//	  {
//	    setMotorA(0);
//	    setMotorB(0);
//	    osDelay(100);
//	  }
	  /* ---- HAND-ROLL CALIBRATION - delete this block when done ---- */
//	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
//	  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
//	  HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);
//	  HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);
//	  for(;;) { osDelay(100); }
//	  //
//	  /* UART BENCH TEST - car stays still. Delete afterwards. */
//	  for(;;) { setMotorA(0); setMotorB(0); osDelay(100); }
//	  //end of UART test

//	  //one time straight line test:
//	  htim8.Instance->CCR2 = SERVOCENTER;
//	  osDelay(3000);                     /* line it up and let go */
//	  encodersZero();
//	  headingTarget = angleNow;
//	  angleMaxDev   = 0.0f;
//	  heading_pid   = 0;                 /* BASELINE - both OFF */
//	  motor_pid     = 0;
//	  float travelled = 0.0f;
//	  int   elapsed   = 0;
//	  setMotorA(DRIVE_SPEED);
//	  setMotorB(DRIVE_SPEED);
//	  while (travelled < (TARGET_CM - MOVE_OVERSHOOT) && elapsed < DRIVE_TIMEOUT)
//	  {
//	      travelled = 0.5f * (distA() + distB());
//	      htim8.Instance->CCR2 = SERVOCENTER;   /* pinned - no PID yet */
//	      osDelay(10);
//	      elapsed += 10;
//	  }
//	  setMotorA(0);
//	  setMotorB(0);
//	  float dCut = 0.5f * (distA() + distB());
//	  osDelay(700);                       /* let it coast fully to a stop */
//	  float dEnd = 0.5f * (distA() + distB());
//	  int ov10 = (int)((dEnd - dCut) * 10.0f);
//	  int en10 = (int)(dEnd * 10.0f);
//	  int p10  = (int)(angleMaxDev * 10.0f);
//	  snprintf(oled_display[4], sizeof(oled_display[4]), "D%d.%d O%d.%d",
//	           en10/10, en10%10, ov10/10, ov10%10);
//	  snprintf(oled_display[0], sizeof(oled_display[0]), "Pk%d.%d END", p10/10, p10%10);
//	  for(;;) { setMotorA(0); setMotorB(0); osDelay(100); }
//	  //end of straight line test

	  for(;;)
	  {
	      /* ---- speed tier from dir magnitude ---- */
	      if      (left_dir ==  1 || left_dir == -1) left_pwmVal_motor = MOTORLOW;
	      else if (left_dir ==  2 || left_dir == -2) left_pwmVal_motor = MOTORMID;
	      else if (left_dir ==  3 || left_dir == -3) left_pwmVal_motor = MOTORHIGH;

	      if      (right_dir ==  1 || right_dir == -1) right_pwmVal_motor = MOTORLOW;
	      else if (right_dir ==  2 || right_dir == -2) right_pwmVal_motor = MOTORMID;
	      else if (right_dir ==  3 || right_dir == -3) right_pwmVal_motor = MOTORHIGH;

	      /* ---- motor PID splits the duty ---- */
	      if (motor_pid == 1)
	      {
	          int l = (int)left_pwmVal_motor  - motorCorrection;
	          int r = (int)right_pwmVal_motor + motorCorrection;
	          if (l > MOTORMAX) l = MOTORMAX;  else if (l < MOTORMIN) l = MOTORMIN;
	          if (r > MOTORMAX) r = MOTORMAX;  else if (r < MOTORMIN) r = MOTORMIN;
	          left_pwmVal_motor  = (uint16_t)l;
	          right_pwmVal_motor = (uint16_t)r;
	      }

	      /* ---- inner wheel slowed during a turn ---- */
	      if (move_dir == 'R') right_pwmVal_motor = (uint16_t)(right_pwmVal_motor * TURNRATIO);
	      if (move_dir == 'L') left_pwmVal_motor  = (uint16_t)(left_pwmVal_motor  * TURNRATIO);

	      /* ---- LEFT motor = A  (his GPIO dir + PWM -> your signed call) ---- */
	      if      (left_dir > 0) setMotorA( (int16_t)left_pwmVal_motor);
	      else if (left_dir < 0) setMotorA(-(int16_t)left_pwmVal_motor);
	      else                   setMotorA(0);

	      /* ---- RIGHT motor = B ---- */
	      if      (right_dir > 0) setMotorB( (int16_t)right_pwmVal_motor);
	      else if (right_dir < 0) setMotorB(-(int16_t)right_pwmVal_motor);
	      else                    setMotorB(0);

	      /* ---- SERVO ---- */
	      if (heading_pid == 1)
	      {
	          int temp_pwmVal_servo = (int)target_pwmVal_servo + headingCorrection;
	          if      (temp_pwmVal_servo > SERVOMAX) temp_pwmVal_servo = SERVOMAX;
	          else if (temp_pwmVal_servo < SERVOMIN) temp_pwmVal_servo = SERVOMIN;
	          pwmVal_servo = (uint16_t)temp_pwmVal_servo;
	      }
	      else
	      {
	          pwmVal_servo = target_pwmVal_servo;
	      }
	      htim8.Instance->CCR2 = pwmVal_servo;

	      osDelay(40);
	  }
  /* USER CODE END motor */
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
	  angleNow     = 0.0f;
	  gyroLastTick = HAL_GetTick();
	  for(;;)
	  {
	    ICM20948_readGyroscope_Z(&hi2c2, 0, GYRO_FULL_SCALE_250DPS, &gyroZ);
	    uint32_t now  = HAL_GetTick();
	    uint32_t dtMs = now - gyroLastTick;
	    float    dt   = dtMs * 0.001f;
	    gyroLastTick  = now;
	    correctedZ = gyroZ - gyroOffset;
	    /* Reject physically impossible rates - a corrupted I2C read comes back
	       saturated, and integrating one produced the old ~200 deg jumps. */
	    if (correctedZ > 250.0f || correctedZ < -250.0f)
	    {
	        gyroBadReads++;
	        correctedZ = 0.0f;
	    }
	    angleNow += (correctedZ - gyroDrift) * dt;
	    headingCorrection = heading_pid_correction(headingTarget, angleNow);
	    float dev = angleNow - headingTarget;
	    if (dev < 0) dev = -dev;
	    if (dev > angleMaxDev) angleMaxDev = dev;
	    snprintf(oled_display[2], sizeof(oled_display[2]),
	             "Ang:%-4d Pk:%-4d", (int)angleNow, (int)angleMaxDev);
	    osDelay(10);
	  }
  /* USER CODE END gyro_task */
}

/* USER CODE BEGIN Header_comm_task */
/**
* @brief Function implementing the Comm_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_comm_task */
/* Telemetry for the RPi: FIN:<dist>,<heading> with no float printf. */
static void sendFin(void)
{
    char buf[32];
    int d10 = (int)(batchDist * 10.0f);
    int h10 = (int)(angleNow  * 10.0f);
    int ad  = d10 < 0 ? -d10 : d10;
    int ah  = h10 < 0 ? -h10 : h10;

    int n = snprintf(buf, sizeof(buf), "FIN:%s%d.%d,%s%d.%d\r\n",
                     d10 < 0 ? "-" : "", ad / 10, ad % 10,
                     h10 < 0 ? "-" : "", ah / 10, ah % 10);
    HAL_UART_Transmit(&huart3, (uint8_t *)buf, n, 100);
}


void comm_task(void *argument)
{
  /* USER CODE BEGIN comm_task */
	  uint8_t cur_inst[FRAME_LEN + 1];
	  int     instr_value = 0;

	  snprintf(oled_display[0], sizeof(oled_display[0]), "Task: idle     ");

	  for(;;)
	  {
	      if (estopFlag) { MotorsOff(); osDelay(20); continue; }

	      if (runRequested == 1)
	      {
	          batchDist = 0.0f;

	          for (uint8_t i = 0; i < instrLen; i++)
	          {
	              if (estopFlag) break;

	              memcpy(cur_inst, (const void *)instrList[i], FRAME_LEN);
	              cur_inst[FRAME_LEN] = '\0';          /* atoi needs a terminator */
	              instr_value = atoi((char *)&cur_inst[2]);

	              switch ((((uint16_t)cur_inst[0]) << 8) | (cur_inst[1] & 0xFF))
	              {
	                  case 'FC': FrontCenter(instr_value); break;
	                  case 'BC': BackCenter(instr_value);  break;
	                  case 'FL': FrontLeft(instr_value);   break;
	                  case 'FR': FrontRight(instr_value);  break;
	                  case 'BL': BackLeft(instr_value);    break;
	                  case 'BR': BackRight(instr_value);   break;
	                  case 'GC': CalGyroDrift(4000); ZeroGyro(); break;
	                  case 'G0': ZeroGyro();               break;
	                  case 'TO': osDelay(10000);           break;   /* fixed 10 s */
	                  default:   break;                             /* FU/BU unsupported */
	              }
	              osDelay(10);
	          }

	          osDelay(200);                /* mechanical settling */
	          sendFin();
	          instrLen = 0;
	          runRequested = 0;
	      }
	      osDelay(20);
	  }
  /* USER CODE END comm_task */
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
