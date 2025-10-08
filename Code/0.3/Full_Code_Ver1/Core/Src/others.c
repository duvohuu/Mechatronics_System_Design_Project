/*
 * others.c
 *
 *  Created on: Aug 29, 2025
 *      Author: Asus
 */


#include "others.h"
#include "stm32f1xx_hal.h"

/**
 * @brief   Functions for controlling motor directions; stopping for receiving package;
 *          as well as checking conditions for stopping at the end of line or turning.
 *
 * @param   adc_readout     Array of ADC values from 5 line sensors [0..4].
 * @param   _w_right        Pointer to the desired angular velocity of the right motor [rad/s].
 * @param   _w_left         Pointer to the desired angular velocity of the left motor [rad/s].
 *
 * @return  None.
 *          Some functions set *_w_right and *_w_left to 0 to stop the robot.
 *          Global state flags are also updated, such as:
 *              - stop                 : robot has stopped.
 *              - outline              : robot is outside of the line.
 *              - has_stop_for_package : robot has stopped for a package.
 *              - is_steering          : robot is preparing to turn.
 *              - has_steered          : robot has finished turning.
 *              - empty, blue_package, red_package, other_package : package status flags.
 *
 * @note    These functions are intended to be called periodically
 *          in the control loop (every CONTROL_SAMP_TIME).
 */


bool has_stop_for_package = false;
bool blue_package = false;
bool red_package = false;
bool other_package = false;
bool empty = true;
bool stop = false;
bool outline = false;
bool is_steering = false;
bool has_steered = false;
bool is_delivery_area = false;


	/* FUNCTIONS CONTROL DIRECTION OF MOTORS */
void CCW_Motor_L(){
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_RESET);
}
void CW_Motor_R(){
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_SET);
}
void CW_Motor_L(){
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_SET);
}
void CCW_Motor_R(){
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_RESET);
}
void stop_Motor_L(){
	HAL_GPIO_WritePin(BI1_GPIO_Port, BI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port, BI2_Pin, GPIO_PIN_SET);
}
void stop_Motor_R(){
	HAL_GPIO_WritePin(AI1_GPIO_Port, AI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port, AI2_Pin, GPIO_PIN_SET);
}



	/*	FUNCTION TO CHECK IF ROBOT HAS REACHED THE TURN */
void check_to_turn(uint16_t* adc_readout,  volatile float *_displacement)
{
	if(empty == false && (adc_readout[0] <= 1000 && adc_readout[1] >= 2500 && adc_readout[2] >= 2500 && adc_readout[3] >= 2500 && adc_readout[4] <= 1000))
	{
		is_steering = true;
		has_steered = false;
	}
}


	/* FUNCTIONS FOR STOP CONDITIONS */
//void stop_for_package(uint16_t* adc_readout, volatile float *_w_right, volatile float *_w_left)
//{
//	if((empty == true && adc_readout[0] <= 1000 && adc_readout[1] >= 2500 && adc_readout[2] >= 2500 && adc_readout[3] >= 2500 && adc_readout[4] <= 1000))
//	{
////		stop = true;
////		*_w_right = 0;
////		*_w_left = 0;
//		has_stop_for_package = true;
////		HAL_Delay(3000);
////		stop = false;
////		empty = false;
//	}
//}

// 0: dark; 1: light. (>1200: dark ; < 1700: light)
//  10001   10011 11001 11000
void stop_for_package_new(uint16_t* adc_readout, volatile float *_v_ref, volatile float *_w_right, volatile float *_w_left)
{
	if(empty == true && adc_readout[0] <= 1700 && adc_readout[1] >= 2500 && adc_readout[2] >= 2500 && adc_readout[3] >= 2500 && adc_readout[4] <= 1700)
	{
		HAL_Delay(450);
		is_delivery_area = true;
		if (*_v_ref <= 5.0)
		{
			*_w_right = 0;
			*_w_left = 0;
			stop = true;
			has_stop_for_package = true;
		}
	}
}


	/* 	STOP WHEN NO LINE */
void stop_no_line(uint16_t* adc_readout, volatile float *_w_right, volatile float *_w_left)
{
	HAL_Delay(5);
	if ((adc_readout[0] <= 2000) && (adc_readout[1] <= 2000) && (adc_readout[2] <= 2000) && (adc_readout[3] <= 2000) && (adc_readout[4] <= 2000))
	{
		stop = true;
		outline = true;
		*_w_right = 0;
		*_w_left = 0;
	}
}


void restart(uint16_t* adc_readout)
{
	if (outline == true && (adc_readout[0] >= 2500 || adc_readout[1] >= 2500 || adc_readout[2] >= 2500 || adc_readout[3] >= 2500 || adc_readout[4] >= 2500)){
		stop = false;
	}
}










