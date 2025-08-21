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
#define AN1_Pin GPIO_PIN_1
#define AN1_GPIO_Port GPIOA
#define AN2_Pin GPIO_PIN_2
#define AN2_GPIO_Port GPIOA
#define AN3_Pin GPIO_PIN_3
#define AN3_GPIO_Port GPIOA
#define AN4_Pin GPIO_PIN_4
#define AN4_GPIO_Port GPIOA
#define AN5_Pin GPIO_PIN_5
#define AN5_GPIO_Port GPIOA
#define EN2_Data1_Pin GPIO_PIN_6
#define EN2_Data1_GPIO_Port GPIOA
#define EN2_Data2_Pin GPIO_PIN_7
#define EN2_Data2_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_10
#define RX_GPIO_Port GPIOB
#define TX_Pin GPIO_PIN_11
#define TX_GPIO_Port GPIOB
#define BI2_Pin GPIO_PIN_12
#define BI2_GPIO_Port GPIOB
#define BI1_Pin GPIO_PIN_13
#define BI1_GPIO_Port GPIOB
#define AI2_Pin GPIO_PIN_14
#define AI2_GPIO_Port GPIOB
#define AI1_Pin GPIO_PIN_15
#define AI1_GPIO_Port GPIOB
#define EN1_Data1_Pin GPIO_PIN_8
#define EN1_Data1_GPIO_Port GPIOA
#define EN1_Data2_Pin GPIO_PIN_9
#define EN1_Data2_GPIO_Port GPIOA
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define PWM_A_Pin GPIO_PIN_8
#define PWM_A_GPIO_Port GPIOB
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
