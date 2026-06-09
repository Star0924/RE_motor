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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define button4_Pin GPIO_PIN_13
#define button4_GPIO_Port GPIOC
#define button4_EXTI_IRQn EXTI15_10_IRQn
#define button5__Pin GPIO_PIN_1
#define button5__GPIO_Port GPIOC
#define button5__EXTI_IRQn EXTI1_IRQn
#define M1_dirA_Pin GPIO_PIN_3
#define M1_dirA_GPIO_Port GPIOC
#define M5_encoderA_Pin GPIO_PIN_0
#define M5_encoderA_GPIO_Port GPIOA
#define M5_encoderB_Pin GPIO_PIN_1
#define M5_encoderB_GPIO_Port GPIOA
#define M2_encoderA_Pin GPIO_PIN_5
#define M2_encoderA_GPIO_Port GPIOA
#define M5_pwm_Pin GPIO_PIN_6
#define M5_pwm_GPIO_Port GPIOA
#define M6_pwm_Pin GPIO_PIN_7
#define M6_pwm_GPIO_Port GPIOA
#define M1_dirB_Pin GPIO_PIN_4
#define M1_dirB_GPIO_Port GPIOC
#define M4_dirA_Pin GPIO_PIN_1
#define M4_dirA_GPIO_Port GPIOB
#define M4_dirB_Pin GPIO_PIN_2
#define M4_dirB_GPIO_Port GPIOB
#define M5_dirA_Pin GPIO_PIN_12
#define M5_dirA_GPIO_Port GPIOB
#define M5_dirB_Pin GPIO_PIN_13
#define M5_dirB_GPIO_Port GPIOB
#define M3_pwmA_Pin GPIO_PIN_14
#define M3_pwmA_GPIO_Port GPIOB
#define M3_pwmB_Pin GPIO_PIN_15
#define M3_pwmB_GPIO_Port GPIOB
#define M2_pwmA_Pin GPIO_PIN_6
#define M2_pwmA_GPIO_Port GPIOC
#define M2_pwmB_Pin GPIO_PIN_7
#define M2_pwmB_GPIO_Port GPIOC
#define M6_dirA_Pin GPIO_PIN_8
#define M6_dirA_GPIO_Port GPIOC
#define M6_dirB_Pin GPIO_PIN_9
#define M6_dirB_GPIO_Port GPIOC
#define M6_encoderA_Pin GPIO_PIN_8
#define M6_encoderA_GPIO_Port GPIOA
#define M6_encoderB_Pin GPIO_PIN_9
#define M6_encoderB_GPIO_Port GPIOA
#define button1_Pin GPIO_PIN_10
#define button1_GPIO_Port GPIOA
#define button1_EXTI_IRQn EXTI15_10_IRQn
#define button2_Pin GPIO_PIN_11
#define button2_GPIO_Port GPIOA
#define button2_EXTI_IRQn EXTI15_10_IRQn
#define button3_Pin GPIO_PIN_12
#define button3_GPIO_Port GPIOA
#define button3_EXTI_IRQn EXTI15_10_IRQn
#define button6_Pin GPIO_PIN_15
#define button6_GPIO_Port GPIOA
#define button6_EXTI_IRQn EXTI15_10_IRQn
#define M1_encoderB_Pin GPIO_PIN_12
#define M1_encoderB_GPIO_Port GPIOC
#define M1_encoderA_Pin GPIO_PIN_2
#define M1_encoderA_GPIO_Port GPIOD
#define M1_encoderA_EXTI_IRQn EXTI2_IRQn
#define M2_encoderB_Pin GPIO_PIN_3
#define M2_encoderB_GPIO_Port GPIOB
#define M3_encoderA_Pin GPIO_PIN_4
#define M3_encoderA_GPIO_Port GPIOB
#define M3_encoderB_Pin GPIO_PIN_5
#define M3_encoderB_GPIO_Port GPIOB
#define M4_encoderA_Pin GPIO_PIN_6
#define M4_encoderA_GPIO_Port GPIOB
#define M4_encoderB_Pin GPIO_PIN_7
#define M4_encoderB_GPIO_Port GPIOB
#define M1_pwm_Pin GPIO_PIN_8
#define M1_pwm_GPIO_Port GPIOB
#define M4_pwm_Pin GPIO_PIN_9
#define M4_pwm_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
