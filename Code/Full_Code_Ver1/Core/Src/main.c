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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_SAMP 0.01f
#define PULSE_2_RPM (60.0f) / (11.0f * 30.0f * 4.0f * T_SAMP)
	/* Params for discrete PID */
#define KP_1 0.9f
#define KI_1 13.5f
#define KP_2 0.9f
#define KI_2 15.5f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	// data buffer for read ADC value from line sensor
#define DATA_LENGTH 5
uint16_t buffer[DATA_LENGTH];
	// Variables for PID Motor 1
int16_t position_motor_1 = 0, old_position_motor_1 = 0;
int pulse_motor_1 = 0;
volatile int16_t e1_motor_1 = 0.0f, e0_motor_1 = 0.0f;
volatile int16_t v1_motor_1 = 0.0f, v0_motor_1 = 0.0f;
volatile int16_t u0_motor_1 = 0.0f, u1_motor_1 = 0.0f;
const float r_motor_1 = -100.0f;
volatile float y_motor_1 = 0.0f;
volatile uint8_t act_motor_1 = 0;

	// Variables for PID Motor 2
int16_t position_motor_2 = 0, old_position_motor_2 = 0;
int pulse_motor_2 = 0;
volatile int16_t e1_motor_2 = 0.0f, e0_motor_2 = 0.0f;
volatile int16_t v1_motor_2 = 0.0f, v0_motor_2 = 0.0f;
volatile int16_t u0_motor_2 = 0.0f, u1_motor_2 = 0.0f;
const float r_motor_2 = 120.0f;
volatile float y_motor_2 = 0.0f;
volatile uint8_t act_motor_2 = 0;

const uint8_t total_value = 255;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		/////////////////////////////////////////////////////////
		/// Process the position values of 2 Motors' Encoders ///
		/////////////////////////////////////////////////////////

		position_motor_1 = 65535 - __HAL_TIM_GET_COUNTER(&htim1);
		pulse_motor_1 = (int16_t)(position_motor_1 - old_position_motor_1);
		old_position_motor_1 = position_motor_1;
		y_motor_1 = pulse_motor_1 * PULSE_2_RPM;

		position_motor_2 = 65535 - __HAL_TIM_GET_COUNTER(&htim3);
		pulse_motor_2 = (int16_t)(position_motor_2 - old_position_motor_2);
		old_position_motor_2 = position_motor_2;
		y_motor_2 = pulse_motor_2 * PULSE_2_RPM;



		////////////////////////////////////
		/// PID Calculation for 2 Motors ///
		////////////////////////////////////

//		e1_motor_1 = e0_motor_1;
//		e0_motor_1 = (int16_t)(r_motor_1 - y_motor_1);
//		v0_motor_1 = (int16_t)(v1_motor_1 + KI * (e1_motor_1 + e0_motor_1));
//		u0_motor_1 = (int16_t)(KP * e0_motor_1 + v0_motor_1);
//		v1_motor_1 = v0_motor_1;
//
//		e1_motor_2 = e0_motor_2;
//		e0_motor_2 = (int16_t)(r_motor_2 - y_motor_2);
//		v0_motor_2 = (int16_t)(v1_motor_2 + KI * (e1_motor_2 + e0_motor_2));
//		u0_motor_2 = (int16_t)(KP * e0_motor_2 + v0_motor_2);
//		v1_motor_2 = v0_motor_2;


		////////////////////////////////////
		/// PID Discrete Calculation for 2 Motors ///
		////////////////////////////////////

		e1_motor_1 = e0_motor_1;
		e0_motor_1 = (int16_t)(r_motor_1 - y_motor_1);
//		v0_motor_1 = (int16_t)(KI_1 * e0_motor_1 * T_SAMP);
		u0_motor_1 = (int16_t)(u1_motor_1 + KP_1 * (e0_motor_1 - e1_motor_1) + KI_1 * e0_motor_1 * T_SAMP);
		u1_motor_1 = u0_motor_1;
//		v1_motor_1 = v0_motor_1;

		e1_motor_2 = e0_motor_2;
		e0_motor_2 = (int16_t)(r_motor_2 - y_motor_2);
//		v0_motor_2 = (int16_t)(KI_1 * e0_motor_1 * T_SAMP);
		u0_motor_2 = (int16_t)(u1_motor_2 + KP_2 * (e0_motor_2 - e1_motor_2) + KI_2 * e0_motor_2 * T_SAMP);
		u1_motor_2 = u0_motor_2;
//		v1_motor_2 = v0_motor_2;



		////////////////////////
		/// Actuate 2 Motors ///
		////////////////////////

		if (u0_motor_1 >= 255)
		{
			act_motor_1 = 255;
		} else if (u0_motor_1 <= -255) {
			act_motor_1 = 255;
		} else
		{
			act_motor_1 = (uint8_t) abs(u0_motor_1);
		}

		if (u0_motor_2 >= 255)
		{
			act_motor_2 = 255;
		} else if (u0_motor_2 <= -255) {
			act_motor_2 = 255;
		} else
		{
			act_motor_2 = (uint8_t) abs(u0_motor_2);
		}


		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)act_motor_1);
		if (u0_motor_1 <= 0)
		{
			HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_SET);
		}

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint16_t)act_motor_2);
		if (u0_motor_2 <= 0)
		{
			HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_SET);
		}
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
