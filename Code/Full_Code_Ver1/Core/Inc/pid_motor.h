/*
 * pid_motor.h
 *
 *  Created on: Aug 23, 2025
 *      Author: Asus
 */

#ifndef INC_PID_MOTOR_H_
#define INC_PID_MOTOR_H_

#include <stdint.h>
#include "params.h"

void update_Motor_Error(volatile float* _pre_err_R, volatile float* _err_R,
		volatile float* _pre_err_L, volatile float* _err_L,
		volatile int16_t* _out_R, volatile int16_t* _pre_out_R,
		volatile int16_t* _out_L, volatile int16_t* _pre_out_L,
		volatile float _w_right, volatile float _w_left,
		volatile float _curr_w_right, volatile float _curr_w_left,
		volatile uint8_t* _actR, volatile uint8_t* _actL);

#endif /* INC_PID_MOTOR_H_ */
