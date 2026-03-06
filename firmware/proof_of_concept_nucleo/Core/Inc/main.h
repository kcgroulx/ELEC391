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
#include "stm32f1xx_hal.h"

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
#define SOLENOID_6_Pin GPIO_PIN_0
#define SOLENOID_6_GPIO_Port GPIOD
#define SOLENOID_5_Pin GPIO_PIN_1
#define SOLENOID_5_GPIO_Port GPIOD
#define SOLENOID_4_Pin GPIO_PIN_0
#define SOLENOID_4_GPIO_Port GPIOC
#define SOLENOID_3_Pin GPIO_PIN_1
#define SOLENOID_3_GPIO_Port GPIOC
#define SOLENOID_2_Pin GPIO_PIN_2
#define SOLENOID_2_GPIO_Port GPIOC
#define SOLENOID_1_Pin GPIO_PIN_3
#define SOLENOID_1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define USER_BUTTON_Pin GPIO_PIN_6
#define USER_BUTTON_GPIO_Port GPIOA
#define LIMIT1_Pin GPIO_PIN_5
#define LIMIT1_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define OPTO_A2_Pin GPIO_PIN_7
#define OPTO_A2_GPIO_Port GPIOC
#define OPTO_B2_Pin GPIO_PIN_8
#define OPTO_B2_GPIO_Port GPIOC
#define ENC_A2_Pin GPIO_PIN_8
#define ENC_A2_GPIO_Port GPIOA
#define ENC_B2_Pin GPIO_PIN_9
#define ENC_B2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
