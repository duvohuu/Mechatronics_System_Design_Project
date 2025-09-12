/*
 * pid_motor.c
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */

#include <math.h>
#include "params.h"
#include "pid_motor.h"


void update_Motor_Error(volatile int16_t* _pre_err_R, volatile int16_t* _err_R,
		volatile int16_t* _pre_iPart_R, volatile int16_t* _iPart_R,
		volatile int16_t* _pre_err_L, volatile int16_t* _err_L,
		volatile int16_t* _pre_iPart_L, volatile int16_t* _iPart_L,
		volatile int16_t* _out_R, volatile int16_t* _pre_out_R,
		volatile int16_t* _out_L, volatile int16_t* _pre_out_L,
		volatile float _w_right, volatile float _w_left,
		volatile float _curr_w_right, volatile float _curr_w_left,
		volatile uint16_t* _actR, volatile uint16_t* _actL)
{
	_w_right *= RADS_2_RPM;
	_w_left  *= RADS_2_RPM;
	_curr_w_right *= RADS_2_RPM;
	_curr_w_left  *= RADS_2_RPM;

	*_pre_err_R = *_err_R;
	*_pre_err_L = *_err_L;
	*_err_R = (int16_t)(_w_right - _curr_w_right);
	*_err_L = (int16_t)(_w_left - _curr_w_left);

	*_iPart_R = (int16_t)(KI_R * *_err_R * MOTOR_SAMP_TIME);
	*_iPart_L = (int16_t)(KI_L * *_err_L * MOTOR_SAMP_TIME);

	*_out_R = (int16_t)(*_pre_out_R + KP_R * (*_err_R - *_pre_err_R) + *_iPart_R);
	*_out_L = (int16_t)(*_pre_out_L + KP_L * (*_err_L - *_pre_err_L) + *_iPart_L);

	*_pre_out_R = *_out_R;
	*_pre_out_L = *_out_L;

	if (*_out_R > 255)
	{
		*_actR = 255;
	} else if (*_out_R < -255)
	{
		*_actR = 255;
	} else
	{
		*_actR = (uint16_t)(*_out_R);
	}

	if (*_out_L > 255)
	{
		*_actL = 255;
	} else if (*_out_L < -255)
	{
		*_actL = 255;
	} else
	{
		*_actL = (uint16_t)(*_out_L);
	}

	return;

}
