/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* OLED pins corrected 2026-08-25: STM_Ref's .ioc labels PE5/PE6/PE7/PE8 as
 * OLED_SCLK/SDA/RESET/DC, but on the actual WHEELTEC C30D board (per its
 * schematic) PE5/PE6 are a motor driver's IN1/IN2 and PE8 is LED3 -- STM_Ref
 * was written for a different physical board. The real OLED header on this
 * board is PD11/PD12/PD13/PD14 (schematic sheet 1, "OLED Display" block).
 * SCLK/SDA/RESET/DC order below is inferred from the connector's physical
 * pin order and common 4-wire-SPI OLED module convention, not confirmed
 * against the module's own silkscreen. */
#define OLED_SCLK_Pin GPIO_PIN_14
#define OLED_SCLK_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define MotorA_AIN2_Pin GPIO_PIN_2
#define MotorA_AIN2_GPIO_Port GPIOA
#define MotorA_AIN1_Pin GPIO_PIN_3
#define MotorA_AIN1_GPIO_Port GPIOA
#define MotorB_CIN2_Pin GPIO_PIN_5
#define MotorB_CIN2_GPIO_Port GPIOC
#define OLED_RESET_Pin GPIO_PIN_12
#define OLED_RESET_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define MotorB_CIN1_Pin GPIO_PIN_12
#define MotorB_CIN1_GPIO_Port GPIOE
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define MotorA_PWM_Pin GPIO_PIN_6
#define MotorA_PWM_GPIO_Port GPIOC
#define MotorB_PWM_Pin GPIO_PIN_8
#define MotorB_PWM_GPIO_Port GPIOC
#define US_Trigger_Pin GPIO_PIN_4
#define US_Trigger_GPIO_Port GPIOB
#define TIM3_CH2_US_Echo_Pin GPIO_PIN_5
#define TIM3_CH2_US_Echo_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
