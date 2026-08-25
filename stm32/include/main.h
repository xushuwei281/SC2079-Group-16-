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
/* Pin map corrected 2026-08-25 against the real WHEELTEC C30D schematic --
 * STM_Ref (this port's source) was written for a different physical board.
 * See stm32/README.md for the full pin table and what changed.
 *
 * Two drive motors are used: MOTORA (left) and MOTORD (right) headers.
 * MOTORB/MOTORC are not populated -- do not reuse their driver/encoder pins
 * for anything else without checking the schematic again. (Right motor was
 * originally paired to MOTORB/TIM3, moved to MOTORD/TIM5 because TIM3
 * conflicted with the ultrasonic sensor's existing echo-capture use -- see
 * HAL_TIM_IC_MspInit/MX_TIM3_Init, now back to ultrasonic-only.)
 *
 * Motor speed control note: every AT8236 driver's VREF pin is hardwired to
 * 3V3 on this board (not GPIO-controlled), so there is no separate speed
 * pin -- speed is set by PWM-ing whichever of IN1/IN2 is the active
 * direction, holding the other low. See motor(), TIM4 (left) and TIM1
 * (right) in main.c. */
#define OLED_SCLK_Pin GPIO_PIN_14
#define OLED_SCLK_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOD
#define OLED_RESET_Pin GPIO_PIN_12
#define OLED_RESET_GPIO_Port GPIOD
#define OLED_DC_Pin GPIO_PIN_11
#define OLED_DC_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
/* Buzzer corrected: STM_Ref/.ioc says PB10, real board wires it to PA8. */
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define US_Trigger_Pin GPIO_PIN_4
#define US_Trigger_GPIO_Port GPIOB
#define TIM3_CH2_US_Echo_Pin GPIO_PIN_5
#define TIM3_CH2_US_Echo_GPIO_Port GPIOB
/* Servo corrected: STM_Ref generated its PWM on TIM1_CH4/PE14, but PE14 is
 * actually MOTORD's IN1 on this board. The real servo signal is PC6, which
 * STM_Ref's firmware called "MotorA_PWM" (TIM8_CH1) -- that name was always
 * wrong; PC6 was never a motor pin here, VREF is hardwired (see above). */
#define Servo_PWM_Pin GPIO_PIN_6
#define Servo_PWM_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
