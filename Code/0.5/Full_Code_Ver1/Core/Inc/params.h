/*
 * params.h
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

#include <stdint.h>

// Parameters for Line PID Controller
extern const float KP;
extern const float KD;
extern const float PID_TAU;
// Store the coefficients of PID controller of Right Wheel
extern const float KP_R;
extern const float KI_R;
// Store the coefficients of PID controller of Left Wheel
extern const float KP_L;
extern const float KI_L;

// Store the Geometric parameters of the Robot
extern const uint16_t L_WHEEL;					// Distance between 2 wheel
extern const uint16_t D_WHEEL;					// Diameter of wheel
extern const uint16_t DIS_2_LineSen;			// Distance from active axis to line sensor

// Desired velocity of robot
extern const float V_REF;					// Desired velocity
extern const float V_REF_TURN;				// velocity in turn


extern const float RADS_2_RPM;					// Transform from rad/s to rpm
extern const float RPM_2_RADS;					// Transform from rpm to rad/s

// Store the Sampling time for executing Lyapunov Control and Motor PID Control
extern const float CONTROL_SAMP_TIME;			// Sampling time for line following controller
extern const float MOTOR_SAMP_TIME;				// Sampling time for PID controller of motors
extern const uint8_t TIME_RATIO;

// Store Conversion constant from Pulse to RPM
extern const float PULSE_2_RPM;

extern const float PATH_PHASE_1_TIME;
extern const float PATH_PHASE_3_TIME;
extern const uint16_t PATH_PHASE_1_RATIO;
extern const uint16_t PATH_PHASE_3_RATIO;

extern const float vR_PHASE_1;
extern const float wR_PHASE_1;

extern const float vR_PHASE_2;
extern const float wR_PHASE_2;

extern const float vR_PHASE_3;
extern const float wR_PHASE_3;


#endif /* INC_PARAMS_H_ */
