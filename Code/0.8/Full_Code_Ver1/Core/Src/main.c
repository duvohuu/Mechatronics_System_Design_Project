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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "params.h"
#include "color_sensor.h"
#include "get_e2_error.h"
#include "pid_line.h"
#include "pid_motor.h"
#include "others.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define T_SAMP 0.01f
#define PULSE_2_RPM (60.0f) / (11.0f * 30.0f * 4.0f * T_SAMP)
#define PI 3.1415926
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Define the buffer for storing ADC readout values
#define DATA_LENGTH 5
uint16_t data_buffer[DATA_LENGTH];
float A0 = 0, A1 = 0, A2 = 0, A3 = 0, A4 = 0;
float sumADCvalues = 0;

// Variables for estimating the Motors' speeds
int16_t position_motor_1 = 0, old_position_motor_1 = 0;
int pulse_motor_1 = 0;

int16_t position_motor_2 = 0, old_position_motor_2 = 0;
int pulse_motor_2 = 0;

// Variables for calculating Lyapunov-based tracking algorithm
volatile float prev_e1 = 0, curr_e1 = 0;
volatile float prev_e2 = 0, curr_e2 = 0;
volatile float prev_e3 = 0, curr_e3 = 0;
volatile float e2_buffer = 0;


volatile float prev_xM = 0, curr_xM = 0;
volatile float prev_yM = 0, curr_yM = 0;

volatile float prev_thetaM = PI, curr_thetaM = PI;

volatile float prev_xC = 0, curr_xC = 0;
volatile float prev_yC = 0, curr_yC = 0;
volatile float prev_thetaC = PI, curr_thetaC = PI;

volatile float prev_xR = 0, curr_xR = 0;
volatile float prev_yR = 0, curr_yR = 0;
volatile float prev_thetaR = PI, curr_thetaR = PI;

volatile float prev_vM = 0, prev_wM = 0;
volatile float vR = 0;
volatile float wR = 0;

volatile uint8_t time_ratio = 0;

// Variables for computing PI Speed Controller
volatile float w_right = 0, w_left = 0;
volatile float curr_w_right = 0, curr_w_left = 0;

volatile float e1R = 0, e0R = 0;
volatile float v1R = 0, v0R = 0;

volatile float e1L = 0, e0L = 0;
volatile float v1L = 0, v0L = 0;

volatile float delta_w = 0;
volatile float pre_Dpart = 0;

volatile int16_t u0R = 0, u1R = 0;
volatile int16_t u0L = 0, u1L = 0;

volatile uint8_t actR = 0, actL = 0;


// Test
volatile float minus_e2 = 0;
volatile uint16_t count = 0;
volatile uint16_t count_decelartion_1 = 0;
volatile uint16_t count_decelartion_2 = 0;

// Variables for encoder count
volatile float real_nr = 0, real_nl = 0;
volatile float real_wr = 0, real_wl = 0;
volatile float real_uM = 0;
volatile float distance = 0;
volatile float total_distance = 0;
float distance_calib = 0;
volatile float displacement = 0;
volatile float v_ref = 0.0f;

// Variables for auto find line when outline and not to endline
volatile bool recovery_mode = false;
volatile bool outline_right = false;
volatile bool outline_left = false;
volatile uint16_t count_outline_right = 0;
volatile uint16_t count_outline_left = 0;
volatile bool turn_right_phase1 = false;
volatile bool turn_right_phase2 = false;
volatile bool turn_left_phase1 = false;
volatile bool turn_left_phase2 = false;
volatile uint16_t count_turn_right_phase2 = 0;
volatile uint16_t count_turn_left_phase2 = 0;
//volatile uint16_t avoidance_count = 0;
//volatile uint16_t avoidance_count_phase_3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*	DIRECTION OF MOTORS	*/
//void CCW_Motor_L(){
//	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_RESET);
//}
//void CW_Motor_R(){
//	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_SET);
//}
//void CW_Motor_L(){
//	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_SET);
//}
//void CCW_Motor_R(){
//	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_RESET);
//}
//void stop_Motor_L(){
//	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_SET);
//}
//void stop_Motor_R(){
//	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_SET);
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2 && stop == false)
	{
		if (time_ratio == 5)
		{
			if (is_steering == false && recovery_mode == false)
			{
					/* Calculate error e2 */
				prev_e2 = curr_e2;
				curr_e2 = calculate_sensor_value_aftercalib(data_buffer);
				minus_e2 = curr_e2 - prev_e2;
				if (empty == false){
					displacement = total_distance - distance_calib;
				}

				if (displacement <= 2900 && (curr_e2 >= 17.0)){
					recovery_mode = true;
					outline_right = true;
					turn_right_phase1 = true;
				}

				if (displacement <= 2900 && (curr_e2 <= -17.0)){
					recovery_mode = true;
					outline_left = true;
					turn_left_phase1 = true;
				}

				if (displacement >= 250 && displacement <= 2250){
					calculate_PID_trackingline(V_REF_TURN, curr_e2, prev_e2, &delta_w, &pre_Dpart, &w_right, &w_left, &displacement);
				}
				else if (is_delivery_area == true && count_decelartion_1 != 13){
					v_ref = V_REF - count_decelartion_1 * 41.65f;
					calculate_PID_trackingline(v_ref, curr_e2, prev_e2, &delta_w, &pre_Dpart, &w_right, &w_left, &displacement);
					count_decelartion_1++;
				}
				else if (displacement >= 2730 && count_decelartion_2 != 21){
					v_ref = V_REF - count_decelartion_2 * (V_REF - 2.0) / 20.0f;
					calculate_PID_trackingline(v_ref, curr_e2, prev_e2, &delta_w, &pre_Dpart, &w_right, &w_left, &displacement);
					count_decelartion_2++;
				}
				else {
					calculate_PID_trackingline(V_REF, curr_e2, prev_e2, &delta_w, &pre_Dpart, &w_right, &w_left, &displacement);
				}


//					/* Calculate output of PID line following controller */
//				calculate_PID_trackingline(curr_e2, prev_e2, &delta_w, &pre_Dpart, &w_right, &w_left, &total_distance, &red_package, &blue_package);

			}
			e2_buffer = calculate_sensor_value_aftercalib(data_buffer);

					/* Check if robot get to turn and turn when count != 10 */
			if (is_steering == true && red_package == true && count != 10 && has_steered == false)
			{
				/* for V_REF = 0.5 m/s */
				w_right = (V_REF_TURN / (D_WHEEL / 2.0f)) + 2.0f;
				w_left = (V_REF_TURN / (D_WHEEL / 2.0f)) - 2.0f;
				/* for V_REF = 0.5 m/s */
//				w_right = 15.0f;
//				w_left = 6.5f;
				count++;
			}


			if (is_steering == true && blue_package == true && count != 10 && has_steered == false)
			{
				/* for V_REF = 0.5 m/s */
				w_right = (V_REF_TURN / (D_WHEEL / 2.0f)) - 2.0f;
				w_left = (V_REF_TURN / (D_WHEEL / 2.0f)) + 2.0f;
				/* for V_REF = 0.5 m/s */
//				w_right = 6.5f;
//				w_left = 15.0f;
				count++;
			}

			if (count == 10)
			{
				has_steered = true;
				is_steering = false;
			}

			time_ratio = 0;
		}
		if (recovery_mode == true){
			if (outline_right == true){
				if (data_buffer[1] > 2000 && turn_right_phase1 == true){
					turn_right_phase1 = false;
					turn_right_phase2 = true;
				}
				if (turn_right_phase1 == true) {
					w_right = (V_REF_TURN / (D_WHEEL / 2.0f)) + 3.5f;
					w_left = (V_REF_TURN / (D_WHEEL / 2.0f)) - 1.5f;
				}
				if (turn_right_phase2 == true){
					w_right = 0.0f;
					w_left = (V_REF_TURN / (D_WHEEL / 2.0f)) - 2.0f;
					count_turn_right_phase2++;
				}
				if (count_turn_right_phase2 >= 40 && (data_buffer[2] > 2800 && abs_float(e2_buffer) < 8.0)){
					recovery_mode = false;
					outline_right = false;
					outline = false;
					turn_right_phase1 = false;
					turn_right_phase2 = false;
					count_turn_right_phase2 = 0;
				}
			}
			if (outline_left == true){
				if (data_buffer[3] > 2000 && turn_left_phase1 == true){
					turn_left_phase1 = false;
					turn_left_phase2 = true;
				}
				if (turn_left_phase1 == true) {
					w_left = (V_REF_TURN / (D_WHEEL / 2.0f)) + 3.5f;
					w_right = (V_REF_TURN / (D_WHEEL / 2.0f)) - 1.5f;
				}
				if (turn_left_phase2 == true){
					w_left = 0.0f;
					w_right = (V_REF_TURN / (D_WHEEL / 2.0f)) - 2.0f;
					count_turn_left_phase2++;
				}
				if (count_turn_left_phase2 >= 40 && (data_buffer[2] > 2800 && abs_float(e2_buffer) < 8.0)){
					recovery_mode = false;
					outline_left = false;
					outline = false;
					turn_left_phase1 = false;
					turn_left_phase2 = false;
					count_turn_left_phase2 = 0;
				}
			}
		}
	}


		/////////////////////////////////////////////////////////
		/// Process the position values of 2 Motors' Encoders ///
		/////////////////////////////////////////////////////////

		position_motor_1 = 65535 - __HAL_TIM_GET_COUNTER(&htim1);
		pulse_motor_1 = (int16_t)(position_motor_1 - old_position_motor_1);
		old_position_motor_1 = position_motor_1;
		curr_w_right = pulse_motor_1 * PULSE_2_RPM * RPM_2_RADS;		// (rad/s)
			// Encoder count for right wheel
		real_nr = (float)(pulse_motor_1 * 60)/(330 * MOTOR_SAMP_TIME);
		real_wr = curr_w_right;

		position_motor_2 = __HAL_TIM_GET_COUNTER(&htim3);
		pulse_motor_2 = (int16_t)(position_motor_2 - old_position_motor_2);
		old_position_motor_2 = position_motor_2;
		curr_w_left = pulse_motor_2 * PULSE_2_RPM * RPM_2_RADS;		// (rad/s)
			// Encoder count for right wheel
		real_nl = (float)(pulse_motor_2 * 60)/(330 * MOTOR_SAMP_TIME);
		real_wl = curr_w_left;

			// Calculate total distance

		real_uM = (real_wr + real_wl) * (D_WHEEL / 4.0f);
		distance = real_uM * MOTOR_SAMP_TIME;
		total_distance += distance / (1.5f);




		////////////////////////////////////
		/// PID Calculation for 2 Motors ///
		////////////////////////////////////

		update_Motor_Error(&e1R, &e0R,
						   &e1L, &e0L,
						   &u0R, &u1R,
						   &u0L, &u1L,
						   w_right, w_left,
						   curr_w_right, curr_w_left,
						   &actR, &actL);



		////////////////////////
		/// Actuate 2 Motors ///
		////////////////////////

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (uint16_t)actR);
		if (u0R <= 0)
		{
			CCW_Motor_R();
		} else {
			CW_Motor_R();
		}

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, (uint16_t)actL);
		if (u0L <= 0)
		{
			CW_Motor_L();
		} else {
			CCW_Motor_L();
		}
		time_ratio ++;
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)data_buffer, DATA_LENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  check_to_turn(data_buffer, &displacement);
//	  stop_for_package(data_buffer, &w_right, &w_left);
	  stop_for_package_new(data_buffer, &v_ref, &w_right, &w_left);
	  stop_no_line(data_buffer, &w_right, &w_left);
	  if (has_stop_for_package == true){
		  distance_calib = total_distance;
		  detect_color();
	  }
	  restart(data_buffer);
	  if (recovery_mode == true && has_stop_for_package == false){
		  stop = false;
	  }
//	  detect_color();
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 280;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BI2_Pin|BI1_Pin|AI2_Pin|AI1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BI2_Pin BI1_Pin AI2_Pin AI1_Pin */
  GPIO_InitStruct.Pin = BI2_Pin|BI1_Pin|AI2_Pin|AI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
