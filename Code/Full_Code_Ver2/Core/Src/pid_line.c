/*
 * pid_line.c
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */


#include "pid_line.h"
#include "params.h"


void update_Error(volatile float* _prev_e1, volatile float* _curr_e1,
		volatile float* _prev_e2, volatile float* _curr_e2,
		volatile float* _prev_e3, volatile float* _curr_e3,
		volatile float _prev_xM, volatile float _curr_xM,
		volatile float _prev_yM, volatile float _curr_yM,
		volatile float _curr_thetaM,
		volatile float* _prev_xC, volatile float* _curr_xC,
		volatile float* _prev_yC, volatile float* _curr_yC,
		volatile float* _prev_thetaC, volatile float* _curr_thetaC,
		volatile float* _prev_xR, volatile float* _curr_xR,
		volatile float* _prev_yR, volatile float* _curr_yR,
		volatile float* _prev_thetaR, volatile float* _curr_thetaR,
		volatile float* _wM)
{
	////////////////////////////////////////////////////////
	//Implement reading from Sensors to assign to _curr_e2//
	////////////////////////////////////////////////////////

	// Calculate the distance made from previous control effort
	float dS = sqrt(pow((_prev_xM - _curr_xM), 2) + pow((_prev_yM - _curr_yM), 2));


	// Update previous e3
	*_prev_e3 = *_curr_e3;
	// Calculate e3 based on the acquired e2
	*_curr_e3 = atan2f(*_curr_e2 - *_prev_e2, dS);


	// Estimate the reference point's Robot self-rotating angular velocity at the current time-stamp
//	*_wR = _prev_wM + (*_curr_e3 - *_prev_e3) / CONTROL_SAMP_TIME;


	// Estimate the reference point's Robot kinematic parameters at the current time-stamp
	float A = tanf(*_curr_e3 + _curr_thetaM);
	float B = *_prev_yR - A * (*_prev_xR);

	// Update the previous position of the Center of Robot's Line tracking system
	*_prev_xC = *_curr_xC;
	*_prev_yC = *_curr_yC;
	*_prev_thetaC = *_curr_thetaC;

	// Update the current position of the Center of Robot's Line tracking system
	*_curr_xC = _curr_xM + DIS_2_LineSen * cosf(_curr_thetaM);
	*_curr_yC = _curr_yM + DIS_2_LineSen * sinf(_curr_thetaM);
	*_curr_thetaC = _curr_thetaM;

	// Update the previous position of the Next Reference point of Robot
	*_prev_xR = *_curr_xR;
	*_prev_yR = *_curr_yR;
	*_prev_thetaR = *_curr_thetaR;

	// Update the current position of the Next Reference point of Robot
	*_curr_xR = (*_curr_e2 - B * cosf(*_curr_thetaC) - (*_curr_xC * sinf(*_curr_thetaC) - *_curr_yC * cosf(*_curr_thetaC))) / (A * cosf(*_curr_thetaC) - sinf(*_curr_thetaC));
	*_curr_yR = A * (*_curr_xR) + B;
	*_curr_thetaR = *_curr_e3 + _curr_thetaM;

	// Update previous e1
	*_prev_e1 = *_curr_e1;
	// Calculate the e1 based on calculations listed above
	*_curr_e1 = (*_curr_xR - *_curr_xC) * cosf(*_curr_thetaC) + (*_curr_yR - *_curr_yC) * sinf(*_curr_thetaC);

	// Update previous e2
	*_prev_e2 = *_curr_e2;

}



void update_M_Values(volatile float* _prev_xM, volatile float* _curr_xM,
					 volatile float* _prev_yM, volatile float* _curr_yM,
					 volatile float* _prev_thetaM, volatile float* _curr_thetaM,
					 volatile float _prev_vM, volatile float _prev_wM)
{
	///////////////////////////////////
	/// Update Kinematic parameters ///
	//////////////////////////////////

	// Update previous position of the Center of Motors' Driving Shaft
	*_prev_xM = *_curr_xM;
	*_prev_yM = *_curr_yM;
	*_prev_thetaM = *_curr_thetaM;

	// Update the current position of the Center of Motors' Driving shaft
	*_curr_xM = *_prev_xM + _prev_vM * cosf(*_prev_thetaM) * CONTROL_SAMP_TIME;
	*_curr_yM = *_prev_yM + _prev_vM * sinf(*_prev_thetaM) * CONTROL_SAMP_TIME;
	*_curr_thetaM = *_prev_thetaM + _prev_wM * CONTROL_SAMP_TIME;
}




void calculate_PID_trackingline(volatile float _e2, volatile float _pre_e2, volatile float* _w_right, volatile float* _w_left)
{
	float delta_u = KP * (_pre_e2 - _e2) + KD * (_pre_e2 - _e2) / CONTROL_SAMP_TIME;
    // Transform block to trans delta_u to desired speed of 2 motor
    *_w_left  = (V_REF - (L_WHEEL / 2.0f)*delta_u) / (D_WHEEL / 2.0f);
    *_w_right = (V_REF + (L_WHEEL / 2.0f)*delta_u) / (D_WHEEL / 2.0f);
    _pre_e2 = _e2;
}



