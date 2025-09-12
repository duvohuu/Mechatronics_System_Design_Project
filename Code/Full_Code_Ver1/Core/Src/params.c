/*
 * params.c
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */


#include "params.h"

const float PI  = 3.1415926;

// Parameters for Line PID Controller
const float KP = 0.075f;
const float KD = 0.018f;

// Store the coefficients of PID controller of Right Wheel
const float KP_R = 1.3f;
const float KI_R = 13.5f;
// Store the coefficients of PID controller of Left Wheel
const float KP_L = 1.3f;
const float KI_L = 13.5f;

// Store the Geometric parameters of the Robot
const uint16_t L_WHEEL = 200;
const uint16_t D_WHEEL = 136;
const uint16_t DIS_2_LineSen = 50;

// Desired velocity of robot
const uint16_t V_REF = 700.0f;

const float RADS_2_RPM = (60.0f / (2 * PI));
const float RPM_2_RADS = 1 / RADS_2_RPM;

const float CONTROL_SAMP_TIME = 0.1f;	// PID line tracking controller sampling time
const float MOTOR_SAMP_TIME = 0.01f;	// PID motor sampling time
const float PULSE_2_RPM = (60.0f) / (11.0f * 30.0f * 4.0f * MOTOR_SAMP_TIME);
const uint8_t TIME_RATIO = (uint8_t)(CONTROL_SAMP_TIME / MOTOR_SAMP_TIME);


const float PATH_PHASE_1_TIME = 1.0f;
const float PATH_PHASE_3_TIME = 1.0f;
const uint16_t PATH_PHASE_1_RATIO = (uint16_t)(PATH_PHASE_1_TIME / MOTOR_SAMP_TIME);
const uint16_t PATH_PHASE_3_RATIO = (uint16_t)(PATH_PHASE_3_TIME / MOTOR_SAMP_TIME);

const float vR_PHASE_1 = 0.0f;
const float wR_PHASE_1 = 0.0f;

const float vR_PHASE_2 = 0.0f;
const float wR_PHASE_2 = 0.0f;

const float vR_PHASE_3 = 0.0f;
const float wR_PHASE_3 = 0.0f;
