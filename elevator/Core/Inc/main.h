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
#define LED_A_F1_Pin GPIO_PIN_14
#define LED_A_F1_GPIO_Port GPIOC
#define LED_A_F2_Pin GPIO_PIN_15
#define LED_A_F2_GPIO_Port GPIOC
#define MOTOR_A_IN4_Pin GPIO_PIN_0
#define MOTOR_A_IN4_GPIO_Port GPIOC
#define MOTOR_A_IN2_Pin GPIO_PIN_1
#define MOTOR_A_IN2_GPIO_Port GPIOC
#define MOTOR_A_IN1_Pin GPIO_PIN_2
#define MOTOR_A_IN1_GPIO_Port GPIOC
#define MOTOR_A_IN3_Pin GPIO_PIN_3
#define MOTOR_A_IN3_GPIO_Port GPIOC
#define LED_A_F3_Pin GPIO_PIN_0
#define LED_A_F3_GPIO_Port GPIOA
#define LED_A_F4_Pin GPIO_PIN_1
#define LED_A_F4_GPIO_Port GPIOA
#define LED_A_F5_Pin GPIO_PIN_4
#define LED_A_F5_GPIO_Port GPIOA
#define LED_B_F1_Pin GPIO_PIN_6
#define LED_B_F1_GPIO_Port GPIOA
#define LED_B_F2_Pin GPIO_PIN_7
#define LED_B_F2_GPIO_Port GPIOA
#define MOTOR_B_IN2_Pin GPIO_PIN_10
#define MOTOR_B_IN2_GPIO_Port GPIOB
#define PI_B_F4_Pin GPIO_PIN_6
#define PI_B_F4_GPIO_Port GPIOC
#define LED_B_F4_Pin GPIO_PIN_7
#define LED_B_F4_GPIO_Port GPIOC
#define PI_B_F2_Pin GPIO_PIN_8
#define PI_B_F2_GPIO_Port GPIOC
#define PI_B_F1_Pin GPIO_PIN_9
#define PI_B_F1_GPIO_Port GPIOC
#define MOTOR_B_IN1_Pin GPIO_PIN_8
#define MOTOR_B_IN1_GPIO_Port GPIOA
#define LED_B_F5_Pin GPIO_PIN_9
#define LED_B_F5_GPIO_Port GPIOA
#define PI_A_F1_Pin GPIO_PIN_10
#define PI_A_F1_GPIO_Port GPIOC
#define PI_A_F2_Pin GPIO_PIN_11
#define PI_A_F2_GPIO_Port GPIOC
#define PI_A_F3_Pin GPIO_PIN_12
#define PI_A_F3_GPIO_Port GPIOC
#define PI_A_F4_Pin GPIO_PIN_2
#define PI_A_F4_GPIO_Port GPIOD
#define MOTOR_B_IN3_Pin GPIO_PIN_4
#define MOTOR_B_IN3_GPIO_Port GPIOB
#define MOTOR_B_IN4_Pin GPIO_PIN_5
#define MOTOR_B_IN4_GPIO_Port GPIOB
#define LED_B_F3_Pin GPIO_PIN_6
#define LED_B_F3_GPIO_Port GPIOB
#define PI_A_F5_Pin GPIO_PIN_7
#define PI_A_F5_GPIO_Port GPIOB
#define PI_B_F3_Pin GPIO_PIN_8
#define PI_B_F3_GPIO_Port GPIOB
#define PI_B_F5_Pin GPIO_PIN_9
#define PI_B_F5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
