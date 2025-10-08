/*
 * pid_line.c
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */


#include "pid_line.h"
#include "params.h"
#include "others.h"


float abs_float(float num)
{
	if (num <= 0)
	{
		return num * -1.0f;
	} else
	{
		return num;
	}
}


void calculate_PID_trackingline(volatile float _v_ref, volatile float _e2, volatile float _pre_e2, volatile float* _delta_w, volatile float* _pre_Dpart, volatile float *_w_right_setpoint, volatile float *_w_left_setpoint, volatile float *_displacement)
{
	*_delta_w = KP * _e2 + KD * (_e2 - _pre_e2) / CONTROL_SAMP_TIME;
//	*_delta_w = KP * _e2 + 2 * KD * (_e2 - _pre_e2) / (2 * PID_TAU + CONTROL_SAMP_TIME)
//			+ (2 * PID_TAU - CONTROL_SAMP_TIME) * *_pre_Dpart / (2 * PID_TAU + CONTROL_SAMP_TIME);
//	*_pre_Dpart = 2 * KD * (_e2 - _pre_e2) / (2 * PID_TAU + CONTROL_SAMP_TIME)
//			+ (2 * PID_TAU - CONTROL_SAMP_TIME) * *_pre_Dpart / (2 * PID_TAU + CONTROL_SAMP_TIME);
	if (abs_float(*_delta_w) >= 2.0f)
	{
		*_delta_w = 2 * (*_delta_w >= 0 ? 1 : -1);
	}


//	if (has_stop_for_package == true && has_steered == false)
	if (*_displacement >= 260 && *_displacement <= 1280)
	{
		if (abs_float(*_delta_w) >= 3.0f)
		{
			*_delta_w = 3.0 * (*_delta_w >= 0 ? 1 : -1);
		}
	    *_w_left_setpoint  = (_v_ref - (L_WHEEL / 2.0f)* *_delta_w) / (D_WHEEL / 2.0f) + 2.0f;
	    *_w_right_setpoint = (_v_ref + (L_WHEEL / 2.0f)* *_delta_w) / (D_WHEEL / 2.0f) - 2.0f;
	}
	else if (*_displacement >= 1420 && *_displacement <= 2280){
		if (abs_float(*_delta_w) >= 2.0f)
		{
			*_delta_w = 2.0 * (*_delta_w >= 0 ? 1 : -1);
		}
	    *_w_left_setpoint  = (_v_ref - (L_WHEEL / 2.0f)* *_delta_w) / (D_WHEEL / 2.0f);
	    *_w_right_setpoint = (_v_ref + (L_WHEEL / 2.0f)* *_delta_w) / (D_WHEEL / 2.0f);
	}
	else
	{
		if (abs_float(*_delta_w) >= 1.5f)
		{
			*_delta_w = 1.5 * (*_delta_w >= 0 ? 1 : -1);
		}
	    *_w_left_setpoint  = (_v_ref - (L_WHEEL / 2.0f)* *_delta_w) / (D_WHEEL / 2.0f);
	    *_w_right_setpoint = (_v_ref + (L_WHEEL / 2.0f)* *_delta_w) / (D_WHEEL / 2.0f);
	}
    return;
}

//+ KD * (_e2 - _pre_e2) / CONTROL_SAMP_TIME

