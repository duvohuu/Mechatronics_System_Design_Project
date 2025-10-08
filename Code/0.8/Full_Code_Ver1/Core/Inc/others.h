/*
 * others.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Asus
 */

#ifndef INC_OTHERS_H_
#define INC_OTHERS_H_

// @brief:	This file contains functions to control direction of 2 motors; flags; and conditional functions at delivery area, stop point and turn.
// @args:
//
//
//
//
//
// @return:
#include <stdbool.h>
#include "main.h"
#include "params.h"
#include "stm32f1xx_hal.h"

extern bool is_delivery_area;
extern bool yes_or_no_goods;
extern bool has_stop_for_package;
extern bool blue_package;
extern bool red_package;
extern bool other_package;
extern bool empty;
extern bool stop;
extern bool outline;
extern bool is_steering;
extern bool has_steered;


	/* FUNCTIONS CONTROL DIRECTION OF MOTORS */
void CCW_Motor_R(void);
void CW_Motor_R(void);
void CCW_Motor_L(void);
void CW_Motor_L(void);
void stop_Motor_R(void);
void stop_Motor_R(void);

	/*	FUNCTION TO CHECK IF ROBOT HAS REACHED THE TURN */
void check_to_turn(uint16_t* adc_readout,  volatile float *_displacement);

	/* FUNCTIONS FOR STOP CONDITIONS */
void stop_for_package(uint16_t* adc_readout, volatile float *_w_right, volatile float *_w_left);
void stop_for_package_new(uint16_t* adc_readout, volatile float *_v_ref, volatile float *_w_right, volatile float *_w_left);
void stop_no_line(uint16_t* adc_readout, volatile float *_w_right, volatile float *_w_left);
void restart(uint16_t* adc_readout);



#endif /* INC_OTHERS_H_ */
