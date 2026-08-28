/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define __GET_ENCODER_TICK_DELTA(_TIMER, LAST_TICK, _DIST) ({ \
    uint32_t CUR_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(_TIMER)) { \
        _DIST = (CUR_TICK <= LAST_TICK) ? \
                LAST_TICK - CUR_TICK : \
                (65535 - CUR_TICK) + LAST_TICK; \
    } else { \
        _DIST = (CUR_TICK >= LAST_TICK) ? \
                CUR_TICK - LAST_TICK : \
                (65535 - LAST_TICK) + CUR_TICK; \
    } \
    LAST_TICK = CUR_TICK; \
})

#define __SET_ENCODER_LAST_TICK(_TIMER, LAST_TICK) ({ \
    LAST_TICK = __HAL_TIM_GET_COUNTER(_TIMER); \
})
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOE
#define BIN2_Pin GPIO_PIN_6
#define BIN2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOE
#define DC_Pin GPIO_PIN_11
#define DC_GPIO_Port GPIOD
#define OLED_REST_Pin GPIO_PIN_12
#define OLED_REST_GPIO_Port GPIOD
#define OLED_SDIN_Pin GPIO_PIN_13
#define OLED_SDIN_GPIO_Port GPIOD
#define OLED_SCLK_Pin GPIO_PIN_14
#define OLED_SCLK_GPIO_Port GPIOD
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_8
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_9
#define AIN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
