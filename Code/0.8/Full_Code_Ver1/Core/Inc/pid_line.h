/*
 * pid_line.h
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */

#ifndef INC_PID_LINE_H_
#define INC_PID_LINE_H_

#include "params.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

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
		volatile float* _wR);
float abs_float(float num);
void calculate_PID_trackingline(volatile float _v_ref, volatile float _e2, volatile float _pre_e2, volatile float* _delta_w, volatile float* _pre_Dpart, volatile float *_w_right_setpoint, volatile float *_w_left_setpoint, volatile float *_displacement);
void update_M_Values(volatile float* _prev_xM, volatile float* _curr_xM,
					 volatile float* _prev_yM, volatile float* _curr_yM,
					 volatile float* _prev_thetaM, volatile float* _curr_thetaM,
					 volatile float _prev_vM, volatile float _prev_wM);


#endif /* INC_PID_LINE_H_ */
