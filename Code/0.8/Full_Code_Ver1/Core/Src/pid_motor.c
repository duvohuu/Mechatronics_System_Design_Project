/*
 * pid_motor.c
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */

#include <math.h>
#include <stdlib.h>
#include "params.h"
#include "pid_motor.h"


	/* FUNCTION CALCULATE PID CONTROLLER FOR TWO MOTORS */
// @brief:	This function calculates output of PID controller for two motors (PWM).
//			This function operates at each interrupt of TIMER2.
// @args:	_err_R, _pre_err_R: error between desired speed of right wheel (calculate from line following controller) and actual speed (be calculated from encoder).
//			_err_L, _pre_err_L: error between desired speed of left wheel (calculate from line following controller) and actual speed (be calculated from encoder).
//			_out_R, _pre_out_R: current output and previous output signal of PID controller for right motor.
// 			_out_L, _pre_out_L: current output and previous output signal of PID controller for left motor.
//			_w_right, _w_left: setpoint of right and left motor.
//			_curr_w_right, _curr_w_left: actual speed of right and left motor.
// @return: _actR: PWM signal for right motor.
//			_actL: PWM signal for left motor.

void update_Motor_Error(volatile float* _pre_err_R, volatile float* _err_R,
		volatile float* _pre_err_L, volatile float* _err_L,
		volatile int16_t* _out_R, volatile int16_t* _pre_out_R,
		volatile int16_t* _out_L, volatile int16_t* _pre_out_L,
		volatile float _w_right, volatile float _w_left,
		volatile float _curr_w_right, volatile float _curr_w_left,
		volatile uint8_t* _actR, volatile uint8_t* _actL)
{
	*_pre_err_R = *_err_R;
	*_pre_err_L = *_err_L;
	*_err_R = (_w_right - _curr_w_right) * RADS_2_RPM;
	*_err_L = (_w_left - _curr_w_left) * RADS_2_RPM;


	*_out_R = (int16_t)(*_pre_out_R + KP_R * (*_err_R - *_pre_err_R) + KI_R * *_err_R * MOTOR_SAMP_TIME);
	*_out_L = (int16_t)(*_pre_out_L + KP_L * (*_err_L - *_pre_err_L) + KI_L * *_err_L * MOTOR_SAMP_TIME);


	*_pre_out_R = *_out_R;
	*_pre_out_L = *_out_L;


	if (*_out_R >= 255 || *_out_R <= -255)
	{
		*_actR = 255 * _w_right / 330;
	} else
	{
		*_actR = (uint8_t)(abs(*_out_R));
	}

	if (*_out_L > 255 || *_out_L <= -255)
	{
		*_actL = 255 * _w_left / 330;
	} else
	{
		*_actL = (uint8_t)(abs(*_out_L));
	}
	return;
}
