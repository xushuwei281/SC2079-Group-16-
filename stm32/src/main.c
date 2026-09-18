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
#include "FreeRTOS.h"
#include "task.h"
#include "oled.h"
#include "ICM20948.h"
#include <stdio.h>
#include <string.h>
#include "pidHeading.h"
#include "pidMotor.h"
#include <math.h>
#include <stdlib.h>    /* atoi, abs */
#include "velocity_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//for servo
#define PWM_PERIOD  1600
#define SERVOCENTER VELOCITY_SERVO_CENTER
#define SERVOLEFT   VELOCITY_SERVO_LEFT   /* calibrated endpoint: ~21-22cm radius */
#define SERVORIGHT  VELOCITY_SERVO_RIGHT  /* calibrated endpoint: ~21-22cm radius */
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
static volatile int16_t velocitySpeed = 0, velocityYaw = 0;
static volatile uint8_t velocityMode = 0;
static volatile uint32_t velocityTick = 0, motionGeneration = 0;
static volatile uint32_t legacyGeneration = 0;
static volatile uint32_t odometryGeneration = 0;
/* Deferred UART status: ISR and motor task must never block on UART. */
static volatile uint8_t stopPending = 0;

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
  .stack_size = 512 * 4,
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
  .stack_size = 256 * 4,
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
/* Definitions for uartTxMutex */
osMutexId_t uartTxMutexHandle;
const osMutexAttr_t uartTxMutex_attributes = {
  .name = "uartTxMutex",
  .attr_bits = osMutexPrioInherit,
  .cb_mem = NULL,
  .cb_size = 0U
};
/* USER CODE BEGIN PV */
float gyroZ = 0.0f;
float gyroOffset = 0.0f;
float correctedZ = 0.0f;
//for heading PID
volatile float headingTarget     = 0.0f;   /* where we want to point   */
volatile int headingCorrection = 0;      /* servo counts to add      */
volatile uint8_t heading_pid     = 0;      /* 1 = PID on, 0 = off      */
float angleMaxDev = 0.0f;
volatile float angleNow = 0.0f;
uint32_t gyroLastTick = 0;
char gyroMsg[32];
float    gyroDrift    = 0.0f;    /* residual walk, deg/s */
uint32_t gyroBadReads = 0;       /* diagnostic counter   */
char     oled_display[6][24];    /* shared display buffer (16 chars visible on screen) */
//for motor
volatile int motorCorrection = 0;
volatile uint8_t motor_pid = 0;

//Motor
int32_t  dist_target = 0;
volatile float left_dist = 0, right_dist = 0;
volatile int8_t left_dir = 0, right_dir = 0;
static volatile float leftSpeedMm = 0.0f, rightSpeedMm = 0.0f;
static volatile int32_t previousCountA = 0;
static volatile uint16_t previousCountB = 0;
uint16_t left_pwmVal_motor = MOTORMID, right_pwmVal_motor = MOTORMID;
volatile int8_t move_dir = 'C';                /* C:Center  L:Left  R:Right */

//Servo
volatile uint16_t target_pwmVal_servo = SERVOCENTER;
volatile uint16_t pwmVal_servo        = SERVOCENTER;

/* Analog IR sensors on H1: PA2/pin 9 and PA3/pin 11. */
volatile uint16_t irSensor1Raw = 0;
volatile uint16_t irSensor2Raw = 0;
volatile uint16_t ir1_cm = 0;
volatile uint16_t ir2_cm = 0;

/* Ultrasonic sensor HC-SR04: PC10 Trig, PC12 Echo */
volatile uint16_t us_cm = 0;
volatile uint32_t us_raw_us = 0;
static volatile uint32_t usValidTick = 0, irValidTick = 0;
static volatile uint8_t usValid = 0, irValid = 0;

/* 2D Pose Estimation (Odometry + Gyro Z Fusion) */
volatile float robot_x_cm = 20.0f;               /* Start zone center (20cm, 20cm), East=0, North=+90deg */
volatile float robot_y_cm = 20.0f;
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
void ZeroGyro(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void IR_ADC_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_ADC1_CLK_ENABLE();

   /* PA2 = ADC1_IN2, PA3 = ADC1_IN3. Analog mode disables digital pulls. */
   GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0; /* PCLK2 / 4 */
   ADC1->CR1 = 0;
   ADC1->CR2 = ADC_CR2_ADON;
   ADC1->SMPR2 = (ADC1->SMPR2 & ~(ADC_SMPR2_SMP2 | ADC_SMPR2_SMP3)) |
                 (ADC_SMPR2_SMP2_2 | ADC_SMPR2_SMP2_1 | ADC_SMPR2_SMP2_0) |
                 (ADC_SMPR2_SMP3_2 | ADC_SMPR2_SMP3_1 | ADC_SMPR2_SMP3_0);
}

static uint16_t IR_ADC_Read(uint32_t channel)
{
   ADC1->SQR1 = 0;                    /* one conversion */
   ADC1->SQR3 = channel & 0x1FU;
   ADC1->SR = 0;
   ADC1->CR2 |= ADC_CR2_SWSTART;
   uint32_t started = HAL_GetTick();
   while ((ADC1->SR & ADC_SR_EOC) == 0U) {
       if (HAL_GetTick() - started > 2U) return 0;
   }
   return (uint16_t)ADC1->DR;
}

static uint16_t IR_RawToCm(uint16_t raw)
{
   if (raw < 100) raw = 100;
   float v = (raw * 3.3f) / 4095.0f;
   if (v < 0.1f) v = 0.1f;
   float d = 27.09039f * powf(v, -1.00210f);
   if (d > 80.0f) d = 80.0f;
   if (d < 10.0f) d = 10.0f;
   return (uint16_t)(d + 0.5f);
}

/* IR1 calibration from measured ADC points: 3100->10 cm, 2250->15 cm,
   1800->20 cm, 1176->30 cm. ADC values decrease as distance increases. */
static uint16_t IR1_RawToCm(uint16_t raw)
{
   float calibrated_cm;

   if (raw >= 3100U)
   {
      calibrated_cm = 10.0f;
   }
   else if (raw >= 2250U)
   {
      calibrated_cm = 10.0f + (3100U - raw) * (5.0f / 850.0f);
   }
   else if (raw >= 1800U)
   {
      calibrated_cm = 15.0f + (2250U - raw) * (5.0f / 450.0f);
   }
   else if (raw >= 1176U)
   {
      calibrated_cm = 20.0f + (1800U - raw) * (10.0f / 624.0f);
   }
   else
   {
      /* Beyond the calibrated 30 cm point, retain the generic curve and
         offset it to meet this calibration continuously at 1176 ADC. */
      calibrated_cm = IR_RawToCm(raw) + 1.0f;
      if (calibrated_cm > 80.0f) calibrated_cm = 80.0f;
   }

   return (uint16_t)(calibrated_cm + 0.5f);
}

static void HCSR04_Init(void)
{
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
   HAL_TIM_Base_Start(&htim6);
}

static uint16_t HCSR04_ReadCm(uint32_t *raw_us)
{
   /* 0. Ensure ECHO (PC12) is LOW before triggering.
         If ECHO was held HIGH by an out-of-range/blind-zone event, wait up to 2 ms to clear. */
   uint16_t t_clear = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
   while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET) {
       if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - t_clear) > 2000) {
           if (raw_us) *raw_us = 0;
           return 0; // Sensor stuck HIGH, skip this cycle
       }
   }

   /* Lock task scheduler during pulse measurement to prevent ~1ms task preemption jitter */
   vTaskSuspendAll();

   /* 1. Send 10 µs trigger pulse on PC10 (TRIG) */
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
   uint16_t trig_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
   while ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - trig_start) < 12) { }
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

   /* 2. Wait for PC12 (ECHO) to go HIGH (timeout ~6 ms = 6000 µs) */
   uint16_t wait_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
   while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_RESET) {
       if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - wait_start) > 6000) {
           xTaskResumeAll();
           if (raw_us) *raw_us = 0;
           return 0; // Trigger timeout
       }
   }

   /* 3. Count duration while PC12 (ECHO) is HIGH (max 25 ms = 25000 µs / ~430 cm) */
   uint16_t echo_start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);
   uint8_t timed_out = 0;
   while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET) {
       if ((uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - echo_start) > 25000) {
           timed_out = 1;
           break;
       }
   }
   uint16_t echo_us = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim6) - echo_start);

   xTaskResumeAll();

   /* If timed out (> 25 ms / ~4.3m), this was a lost echo, not a valid 400cm reading */
   if (timed_out) {
       if (raw_us) *raw_us = 0;
       return 0;
   }

   if (raw_us) *raw_us = (uint32_t)echo_us;

   /* 4. Convert duration to distance: speed of sound = 0.0343 cm/µs.
         Round trip distance = (echo_us * 0.0343) / 2 = echo_us / 58.2 */
   float dist_cm = (float)echo_us / 58.2f;
   if (dist_cm > 400.0f) dist_cm = 400.0f;
   return (uint16_t)(dist_cm + 0.5f);
}

//Encoder
static int32_t encoderA(void) { return (int32_t)__HAL_TIM_GET_COUNTER(&htim2); }
//motor
#define MOTOR_PPR     	1527.0f	//1320.0f
#define WHEEL_D_CM       6.5f
#define CM_PER_COUNT  (WHEEL_D_CM * 3.1415f / MOTOR_PPR)
static void encodersZero(void)
{
   uint32_t primask = __get_PRIMASK();
   __disable_irq();
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   __HAL_TIM_SET_COUNTER(&htim3, 0);
   previousCountA = 0;
   previousCountB = 0;
   left_dist = right_dist = 0.0f;
   leftSpeedMm = rightSpeedMm = 0.0f;
   __set_PRIMASK(primask);
}
static void setMotorA(int16_t speed)
{
   uint32_t primask = __get_PRIMASK();
   __disable_irq();
   if (estopFlag) speed = 0;
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
   __set_PRIMASK(primask);
}
static void setMotorB(int16_t speed)
{
   uint32_t primask = __get_PRIMASK();
   __disable_irq();
   if (estopFlag) speed = 0;
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
   __set_PRIMASK(primask);
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
  IR_ADC_Init();
  HCSR04_Init();
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
  uartTxMutexHandle = osMutexNew(&uartTxMutex_attributes);
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

  /*Configure Ultrasonic TRIG Pin : PC10 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure Ultrasonic ECHO Pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Thread-safe UART transmission guarded by FreeRTOS mutex
static HAL_StatusTypeDef UART_SafeTransmit(const uint8_t *pData, uint16_t Size, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    if (uartTxMutexHandle != NULL && osKernelGetState() == osKernelRunning) {
        if (osMutexAcquire(uartTxMutexHandle, timeout_ms) == osOK) {
            status = HAL_UART_Transmit(&huart3, (uint8_t *)pData, Size, timeout_ms);
            osMutexRelease(uartTxMutexHandle);
        } else {
            return HAL_BUSY;
        }
    } else {
        status = HAL_UART_Transmit(&huart3, (uint8_t *)pData, Size, timeout_ms);
    }
    return status;
}

/* Stream live telemetry to RPi: TLM:<x_cm>,<y_cm>,<world_deg>,<us_cm>,<ir1_cm>,<ir2_cm> */
static void sendTlm(void)
{
    char buf[64];
    int x10 = (int)(robot_x_cm * 10.0f);
    int y10 = (int)(robot_y_cm * 10.0f);
    float world_deg = 90.0f + angleNow;
    int h10 = (int)(world_deg   * 10.0f);
    int ax  = x10 < 0 ? -x10 : x10;
    int ay  = y10 < 0 ? -y10 : y10;
    int ah  = h10 < 0 ? -h10 : h10;

    int n = snprintf(buf, sizeof(buf), "TLM:%s%d.%d,%s%d.%d,%s%d.%d,%u,%u,%u\r\n",
                     x10 < 0 ? "-" : "", ax / 10, ax % 10,
                     y10 < 0 ? "-" : "", ay / 10, ay % 10,
                     h10 < 0 ? "-" : "", ah / 10, ah % 10,
                     (unsigned int)us_cm, (unsigned int)ir1_cm, (unsigned int)ir2_cm);
    UART_SafeTransmit((const uint8_t *)buf, n, 20);
}

static void MotorsOff(void)
{
    /* CCR 0 on both channels = both driver pins HIGH = brake (polarity LOW) */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    target_pwmVal_servo = SERVOCENTER;
    pwmVal_servo        = SERVOCENTER;
    htim8.Instance->CCR2 = SERVOCENTER;

    /* CRITICAL: otherwise motor() re-drives them 40 ms later */
    left_dir = 0;  right_dir = 0;
    motor_pid = 0; heading_pid = 0;
}

static void EStop(void)
{
    estopFlag = 1;
    velocitySpeed = velocityYaw = 0;
    motionGeneration++;
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
   if (pkt[0] == 'R' && pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0) {
       motionGeneration++;
       velocitySpeed = velocityYaw = 0;
       velocityMode = 0;
       stopPending = 0;
       estopFlag = 0;
       runRequested = 0;
       instrLen = 0;
       MotorsOff();
       ZeroGyro();
       encodersZero();
       snprintf(oled_display[0], sizeof(oled_display[0]), "RESET OK       ");
       return;
   }
   if (estopFlag) return;                       /* latched until reboot or 'R' reset */

   if (pkt[0] == 'Q' && pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0) {
       EStop();
   }
   else if (pkt[0] == 'V') {
       int16_t speed, yaw;
       if (!velocity_decode(pkt, &speed, &yaw)) {
           EStop();
           stopPending = 4; /* Malformed/out-of-bounds target must not retain motion. */
           return;
       }
       velocitySpeed = speed;
       velocityYaw = yaw;
       velocityTick = HAL_GetTick();
       velocityMode = 1;
       motionGeneration++;
       runRequested = 0;
       instrLen = 0;
       if (speed == 0) MotorsOff();
       return;
   }
   else if (runRequested == 1) {
       /* Batch is actively executing; ignore incoming moves to protect current batch */
   }
   else if (pkt[0] == '#' && pkt[1] == 0 && pkt[2] == 0 && pkt[3] == 0 && pkt[4] == 0) {
       if (instrLen == 0 || velocitySpeed != 0) return;
       runRequested = 1;
       if (Comm_taskHandle != NULL) {
           osThreadFlagsSet(Comm_taskHandle, 0x01);
       }
   }
   else if (instrLen >= 40) {
       /* Instruction buffer full; drop to prevent overflow */
   }
   else {
       int maintenance = (pkt[0] == 'G' && (pkt[1] == 'C' || pkt[1] == '0')) ||
                         (pkt[0] == 'T' && pkt[1] == 'O');
       int movement = (pkt[0] == 'F' || pkt[0] == 'B') &&
                      (pkt[1] == 'C' || pkt[1] == 'L' || pkt[1] == 'R');
       if ((!maintenance && !movement) || (velocityMode && !maintenance) ||
           velocitySpeed != 0 || pkt[2] < '0' || pkt[2] > '9' ||
           pkt[3] < '0' || pkt[3] > '9' || pkt[4] < '0' || pkt[4] > '9') return;
       memcpy((void *)instrList[instrLen++], pkt, FRAME_LEN);
       /* no ACK - the RPi protocol does not expect one */
   }

   snprintf(oled_display[0], sizeof(oled_display[0]), "Rx:%.5s N%-2d C%-4lu",
            (char *)pkt, (int)instrLen, (unsigned long)rxCount);
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

static int legacyActive(void)
{
    return !estopFlag && !velocityMode && runRequested &&
           legacyGeneration == motionGeneration;
}

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
    osDelay(50);
    if (!legacyActive()) return;
    right_dir = 2;  left_dir = 2;      /* MOTORMID - cruise */

    int creep_dist = (dist_target > 15) ? 5 : 2;
    uint32_t t0 = osKernelGetTickCount();
    uint32_t timeout_ticks = (uint32_t)(dist * 80 + 2000);

    while (legacyActive() && cur_dist < dist_target - creep_dist && (osKernelGetTickCount() - t0) < timeout_ticks) {
        osDelay(20);
        if (!legacyActive()) return;
        cur_dist = (int)((left_dist + right_dist) / 2.0f);
    }

    right_dir = 1;  left_dir = 1;      /* creep */
    while (legacyActive() && ((left_dist + right_dist) / 2.0f) < dist_target - MOVEOVERSHOOT && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(10);
    if (!legacyActive()) return;

    motor_pid = 0;  heading_pid = 0;
    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    batchDist += 0.5f * (left_dist + right_dist);
    osDelay(20);
    if (!legacyActive()) return;
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
    osDelay(50);
    if (!legacyActive()) return;
    right_dir = -2;  left_dir = -2;

    int creep_dist = (dist > 15) ? 5 : 2;
    uint32_t t0 = osKernelGetTickCount();
    uint32_t timeout_ticks = (uint32_t)(dist * 80 + 2000);

    while (legacyActive() && cur_dist > dist_target + creep_dist && (osKernelGetTickCount() - t0) < timeout_ticks) {
        osDelay(20);
        if (!legacyActive()) return;
        cur_dist = (int)((left_dist + right_dist) / 2.0f);
    }

    right_dir = -1;  left_dir = -1;
    while (legacyActive() && ((left_dist + right_dist) / 2.0f) > dist_target + MOVEOVERSHOOT && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(10);
    if (!legacyActive()) return;

    motor_pid = 0;  heading_pid = 0;
    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    batchDist += 0.5f * (left_dist + right_dist);
    osDelay(20);
    if (!legacyActive()) return;
}

void FrontRight(int angle)
{
    move_dir = 'R';
    motor_pid = 0;  heading_pid = 0;        /* PIDs OFF for the whole turn */

    target_pwmVal_servo = SERVORIGHT;
    osDelay(80);                             /* let the servo physically arrive (~60-80ms throw) */
    if (!legacyActive()) return;

    float startAngle = angleNow;
    right_dir = 2;  left_dir = 2;            /* cruise speed */

    int creep_angle = (angle > 15) ? 4 : 1;
    uint32_t t0 = osKernelGetTickCount();
    uint32_t timeout_ticks = (uint32_t)(angle * 60 + 2000);

    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - creep_angle) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(15);
    if (!legacyActive()) return;

    right_dir = 1;  left_dir = 1;            /* creep */
    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(10);
    if (!legacyActive()) return;

    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(40);
    if (!legacyActive()) return;
}

void FrontLeft(int angle)
{
    move_dir = 'L';
    motor_pid = 0;  heading_pid = 0;

    target_pwmVal_servo = SERVOLEFT;
    osDelay(80);                             /* let the servo physically arrive (~60-80ms throw) */
    if (!legacyActive()) return;

    float startAngle = angleNow;
    right_dir = 2;  left_dir = 2;

    int creep_angle = (angle > 15) ? 4 : 1;
    uint32_t t0 = osKernelGetTickCount();
    uint32_t timeout_ticks = (uint32_t)(angle * 60 + 2000);

    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - creep_angle) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(15);
    if (!legacyActive()) return;

    right_dir = 1;  left_dir = 1;
    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(10);
    if (!legacyActive()) return;

    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(40);
    if (!legacyActive()) return;
}

void BackRight(int angle)
{
    move_dir = 'R';
    motor_pid = 0;  heading_pid = 0;

    target_pwmVal_servo = SERVORIGHT;
    osDelay(80);
    if (!legacyActive()) return;

    float startAngle = angleNow;
    right_dir = -2;  left_dir = -2;

    int creep_angle = (angle > 15) ? 4 : 1;
    uint32_t t0 = osKernelGetTickCount();
    uint32_t timeout_ticks = (uint32_t)(angle * 60 + 2000);

    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - creep_angle) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(15);
    if (!legacyActive()) return;

    right_dir = -1;  left_dir = -1;
    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(10);
    if (!legacyActive()) return;

    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(40);
    if (!legacyActive()) return;
}

void BackLeft(int angle)
{
    move_dir = 'L';
    motor_pid = 0;  heading_pid = 0;

    target_pwmVal_servo = SERVOLEFT;
    osDelay(80);
    if (!legacyActive()) return;

    float startAngle = angleNow;
    right_dir = -2;  left_dir = -2;

    int creep_angle = (angle > 15) ? 4 : 1;
    uint32_t t0 = osKernelGetTickCount();
    uint32_t timeout_ticks = (uint32_t)(angle * 60 + 2000);

    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - creep_angle) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(15);
    if (!legacyActive()) return;

    right_dir = -1;  left_dir = -1;
    while (legacyActive() && fabsf(angleNow - startAngle) < (angle - TURNOVERSHOOT) && (osKernelGetTickCount() - t0) < timeout_ticks)
        osDelay(10);
    if (!legacyActive()) return;

    right_dir = 0;  left_dir = 0;
    target_pwmVal_servo = SERVOCENTER;
    headingTarget = angleNow;
    osDelay(40);
    if (!legacyActive()) return;
}

void CalGyroDrift(int timeCal)
{
    float    old = angleNow;
    uint32_t t0  = HAL_GetTick();
    uint32_t generation = motionGeneration;
    while (HAL_GetTick() - t0 < (uint32_t)timeCal) {
        if (estopFlag || generation != motionGeneration) return;
        osDelay(10);
    }
    float dt = (HAL_GetTick() - t0) * 0.001f;
    gyroDrift += (angleNow - old) / dt;      /* += refines on repeat calls */
}

void ZeroGyro(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    odometryGeneration++;
    angleNow      = 0.0f;
    headingTarget = 0.0f;
    robot_x_cm    = 20.0f;
    robot_y_cm    = 20.0f;
    __set_PRIMASK(primask);
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
  uint8_t oled_div = 0;
  for(;;)
  {
    irSensor1Raw = IR_ADC_Read(2U); /* PA2 / ADC1_IN2 */
    irSensor2Raw = IR_ADC_Read(3U); /* PA3 / ADC1_IN3 */

    ir1_cm = irSensor1Raw ? IR1_RawToCm(irSensor1Raw) : 0;
    ir2_cm = irSensor2Raw ? IR_RawToCm(irSensor2Raw) : 0;
    /* Freshness tracks the acquisition task, not whether a sensor returned a
       positive range. A zero is the established no-return/out-of-range value. */
    irValidTick = HAL_GetTick();
    irValid = 1;

    uint32_t raw_echo = 0;
    us_cm = HCSR04_ReadCm(&raw_echo);
    us_raw_us = raw_echo;
    usValidTick = HAL_GetTick();
    usValid = 1;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t reason = stopPending;
    stopPending = 0;
    __set_PRIMASK(primask);
    const char *status = reason == 1 ? "STOP:PROXIMITY\r\n" :
                         reason == 2 ? "STOP:SENSOR_STALE\r\n" :
                         reason == 3 ? "STOP:WATCHDOG\r\n" :
                         reason == 4 ? "STOP:INVALID_VELOCITY\r\n" : NULL;
    if (status != NULL && UART_SafeTransmit((const uint8_t *)status, strlen(status), 20) != HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        if (!stopPending) stopPending = reason;
        __set_PRIMASK(primask);
    }

    sendTlm();

    if (++oled_div >= 3) {
        oled_div = 0;
        snprintf(oled_display[3], sizeof(oled_display[3]),
                 "US :%-3ucm R:%-5lu ", (unsigned int)us_cm, (unsigned long)us_raw_us);
        snprintf(oled_display[4], sizeof(oled_display[4]),
                 "I1 :%-3ucm R:%-5u ", (unsigned int)ir1_cm, (unsigned int)irSensor1Raw);
        snprintf(oled_display[5], sizeof(oled_display[5]),
                 "I2 :%-3ucm R:%-5u ", (unsigned int)ir2_cm, (unsigned int)irSensor2Raw);

        OLED_ShowString(0,  0, (uint8_t *)oled_display[0]);
        OLED_ShowString(0, 10, (uint8_t *)oled_display[1]);
        OLED_ShowString(0, 20, (uint8_t *)oled_display[2]);
        OLED_ShowString(0, 30, (uint8_t *)oled_display[3]);
        OLED_ShowString(0, 40, (uint8_t *)oled_display[4]);
        OLED_ShowString(0, 50, (uint8_t *)oled_display[5]);
        OLED_Refresh_Gram();
    }
    osDelay(50);
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
		  uint32_t previousTick = HAL_GetTick();
          for(;;)
		  {
              uint32_t now = HAL_GetTick();
              float dt = (now - previousTick) * 0.001f;
              previousTick = now;
              uint32_t primask = __get_PRIMASK();
              __disable_irq();
              uint32_t odometry_generation = odometryGeneration;
              int32_t countA = encoderA();
              uint16_t countB = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
              int32_t deltaA = (int32_t)((uint32_t)countA - (uint32_t)previousCountA);
              int16_t deltaB = (int16_t)(countB - previousCountB);
              previousCountA = countA;
              previousCountB = countB;
              float dL = deltaA * CM_PER_COUNT;
              float dR = -deltaB * CM_PER_COUNT;
              left_dist += dL;
              right_dist += dR;
              if (dt > 0.0f) {
                  leftSpeedMm += 0.35f * (dL * 10.0f / dt - leftSpeedMm);
                  rightSpeedMm += 0.35f * (dR * 10.0f / dt - rightSpeedMm);
              }
              __set_PRIMASK(primask);

		      float delta_s = 0.5f * (dL + dR);

		      /* In arena convention: East=0 rad, North=+90 deg, increasing
		         CCW. Car starts facing North (angleNow=0 -> 90 deg).
		         Subtract, not add: confirmed empirically on real hardware
		         (gyro zeroed, physical right/CW turn: angleNow went 0 -> +3,
		         i.e. angleNow increases for a CW turn) -- 90+angleNow would
		         integrate robot_x_cm/y_cm in the mirrored direction. This is
		         a DIFFERENT local variable from sendTlm()'s world_deg
		         (still 90+angleNow, deliberately) -- the Pi corrects THAT
		         one on receipt (see serial_bridge_node.py), so don't
		         "fix" it to match this formula or heading will invert again. */
		      float world_angle_deg = 90.0f - angleNow;
		      float world_angle_rad = world_angle_deg * (3.1415926535f / 180.0f);

              float dx = delta_s * cosf(world_angle_rad);
              float dy = delta_s * sinf(world_angle_rad);
              primask = __get_PRIMASK();
              __disable_irq();
              if (odometry_generation == odometryGeneration) {
                  robot_x_cm += dx;
                  robot_y_cm += dy;
              }
              __set_PRIMASK(primask);

		      motorCorrection = motor_pid_correction(left_dist, right_dist);

		      snprintf(oled_display[1], sizeof(oled_display[1]),
		               "X:%-3d Y:%-3d A:%-3d", (int)robot_x_cm, (int)robot_y_cm, (int)world_angle_deg);

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
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    MotorsOff();
    VelocityPI leftPI = {0}, rightPI = {0};
    uint32_t previousTick = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();
        float dt = velocity_clamp((now - previousTick) * 0.001f, 0.001f, 0.1f);
        previousTick = now;
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        int forward = velocityMode ? velocitySpeed > 0 :
                      (legacyActive() && (left_dir > 0 || right_dir > 0));
        int safety = velocity_safety(forward, now, usValidTick, irValidTick,
                                     usValid, irValid, us_cm, ir1_cm, ir2_cm);
        if (!estopFlag && safety) {
            EStop();
            stopPending = (uint8_t)safety;
        }
        if (velocityMode && velocity_watchdog_expired(velocitySpeed, now, velocityTick)) {
            velocitySpeed = velocityYaw = 0;
            motionGeneration++;
            MotorsOff();
            stopPending = 3;
        }
        int16_t speed = velocitySpeed, yaw = velocityYaw;
        uint32_t generation = motionGeneration;
        int velocity = velocityMode;
        int stopped = estopFlag;
        __set_PRIMASK(primask);

        if (stopped) {
            MotorsOff();
            leftPI = (VelocityPI){0};
            rightPI = (VelocityPI){0};
        } else if (velocity) {
            float leftTarget, rightTarget;
            uint16_t servo;
            velocity_targets(speed, yaw, &leftTarget, &rightTarget, &servo);
            int16_t leftPWM = velocity_pi(&leftPI, leftTarget, leftSpeedMm, dt);
            int16_t rightPWM = velocity_pi(&rightPI, rightTarget, rightSpeedMm, dt);
            /* Q/R/new V can arrive during floating-point calculations. Commit only
             * the still-current command; never overwrite an interrupt's stop. */
            primask = __get_PRIMASK();
            __disable_irq();
            if (!estopFlag && velocityMode && generation == motionGeneration) {
                setMotorA(leftPWM);
                setMotorB(rightPWM);
                htim8.Instance->CCR2 = servo;
            }
            __set_PRIMASK(primask);
        } else {
            leftPI = (VelocityPI){0};
            rightPI = (VelocityPI){0};
            primask = __get_PRIMASK();
            __disable_irq();
            if (legacyActive()) {
                int leftPWM = abs(left_dir) == 3 ? MOTORHIGH :
                              (abs(left_dir) == 2 ? MOTORMID : MOTORLOW);
                int rightPWM = abs(right_dir) == 3 ? MOTORHIGH :
                               (abs(right_dir) == 2 ? MOTORMID : MOTORLOW);
                if (motor_pid) {
                    leftPWM = (int)velocity_clamp(leftPWM - motorCorrection, MOTORMIN, MOTORMAX);
                    rightPWM = (int)velocity_clamp(rightPWM + motorCorrection, MOTORMIN, MOTORMAX);
                }
                if (move_dir == 'L') leftPWM = (int)(leftPWM * TURNRATIO);
                if (move_dir == 'R') rightPWM = (int)(rightPWM * TURNRATIO);
                setMotorA(left_dir == 0 ? 0 : (left_dir > 0 ? leftPWM : -leftPWM));
                setMotorB(right_dir == 0 ? 0 : (right_dir > 0 ? rightPWM : -rightPWM));
                htim8.Instance->CCR2 = (uint16_t)velocity_clamp(
                    target_pwmVal_servo + (heading_pid ? headingCorrection : 0),
                    SERVOMIN, SERVOMAX);
            } else {
                MotorsOff();
            }
            __set_PRIMASK(primask);
        }
        osDelay(10);
    }
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
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        angleNow += (correctedZ - gyroDrift) * dt;
        __set_PRIMASK(primask);
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
static void sendFin(void)
{
    char buf[64];
    int x10 = (int)(robot_x_cm * 10.0f);
    int y10 = (int)(robot_y_cm * 10.0f);
    float world_deg = 90.0f + angleNow;
    int h10 = (int)(world_deg   * 10.0f);
    int ax  = x10 < 0 ? -x10 : x10;
    int ay  = y10 < 0 ? -y10 : y10;
    int ah  = h10 < 0 ? -h10 : h10;

    int n = snprintf(buf, sizeof(buf), "FIN:POS,%s%d.%d,%s%d.%d,%s%d.%d,%u,%u,%u\r\n",
                     x10 < 0 ? "-" : "", ax / 10, ax % 10,
                     y10 < 0 ? "-" : "", ay / 10, ay % 10,
                     h10 < 0 ? "-" : "", ah / 10, ah % 10,
                     (unsigned int)us_cm, (unsigned int)ir1_cm, (unsigned int)ir2_cm);
    UART_SafeTransmit((const uint8_t *)buf, n, 100);
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

	      osThreadFlagsWait(0x01, osFlagsWaitAny, 20);

	      if (runRequested == 1)
	      {
	          legacyGeneration = motionGeneration;
	          uint32_t generation = legacyGeneration;
	          UART_SafeTransmit((const uint8_t *)"RUN\r\n", 5, 50);
	          batchDist = 0.0f;

	          for (uint8_t i = 0; i < instrLen; i++)
	          {
	              if (estopFlag || generation != motionGeneration) break;

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
	                  case 'GC':
	                      CalGyroDrift(4000);
	                      if (!estopFlag && generation == motionGeneration) ZeroGyro();
	                      break;
	                  case 'G0': ZeroGyro();               break;
	                  case 'TO': {
	                      uint32_t started = HAL_GetTick();
	                      while (!estopFlag && generation == motionGeneration &&
	                             HAL_GetTick() - started < 10000U) osDelay(10);
	                      break;
	                  }
	                  default:   break;                             /* FU/BU unsupported */
	              }
	              osDelay(10);
	          }

	          osDelay(20);                 /* mechanical settling */
	          if (!estopFlag && generation == motionGeneration) {
	              sendFin();
	              instrLen = 0;
	              runRequested = 0;
	          }
	      }
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
