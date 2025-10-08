/*
 * get_e2_error.h
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */

#ifndef INC_GET_E2_ERROR_H_
#define INC_GET_E2_ERROR_H_

#include <stdint.h>
#include <stdlib.h>

#define Y_MAX 3200 		// Maximum desired ADC value of all sensor
#define Y_MIN 175		// Minimum desired ADC value of all sensor
#define LINE_COE_1 1.3028f
#define LINE_COE_2 0.3184f


extern const uint16_t X_MAX[5];		// Array contains maximum ADC value of 5 sensor
extern const uint16_t X_MIN[5];		// Array contains minimum ADC value of 5 sensor
extern volatile uint16_t ADCValues_AfterCalib[5];
extern float sum_ADCvalues;
extern float A0, A1, A2, A3, A4;
extern volatile float e2;


volatile float calculate_sensor_value_aftercalib(uint16_t* adc_readout);

#endif /* INC_GET_E2_ERROR_H_ */
