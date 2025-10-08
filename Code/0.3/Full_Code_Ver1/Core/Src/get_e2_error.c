/*
 * get_e2_error.c
 *
 *  Created on: Aug 22, 2025
 *      Author: Asus
 */


#include "get_e2_error.h"

// @brief:	This function calculates error e2 which from center of array line sensor to center of line.
//			This function operates at each CONTROL_SAMP_TIME.
// @args:	adc_readout: Values of 5 line sensor.
// @return: error e2.


const uint16_t X_MAX[] = {3480, 3220, 3485, 3420, 3450};
const uint16_t X_MIN[] = {214, 203, 289, 259, 350};
volatile uint16_t ADCValues_AfterCalib[5] = {0, 0, 0, 0, 0};
float sum_ADCvalues = 0;

volatile float calculate_sensor_value_aftercalib(uint16_t* adc_readout)
{
	  A0 = (float)(Y_MAX - Y_MIN) / (X_MAX[0] - X_MIN[0]);
	  A1 = (float)(Y_MAX - Y_MIN) / (X_MAX[1] - X_MIN[1]);
	  A2 = (float)(Y_MAX - Y_MIN) / (X_MAX[2] - X_MIN[2]);
	  A3 = (float)(Y_MAX - Y_MIN) / (X_MAX[3] - X_MIN[3]);
	  A4 = (float)(Y_MAX - Y_MIN) / (X_MAX[4] - X_MIN[4]);

	  ADCValues_AfterCalib[0] = (uint16_t)(Y_MIN + A0 * (adc_readout[0] - X_MIN[0])); //	ADC VALUES AFTER CALIB
	  ADCValues_AfterCalib[1] = (uint16_t)(Y_MIN + A1 * (adc_readout[1] - X_MIN[1]));
	  ADCValues_AfterCalib[2] = (uint16_t)(Y_MIN + A2 * (adc_readout[2] - X_MIN[2]));
	  ADCValues_AfterCalib[3] = (uint16_t)(Y_MIN + A3 * (adc_readout[3] - X_MIN[3]));
	  ADCValues_AfterCalib[4] = (uint16_t)(Y_MIN + A4 * (adc_readout[4] - X_MIN[4]));

	  sum_ADCvalues = (ADCValues_AfterCalib[0] + ADCValues_AfterCalib[1] + ADCValues_AfterCalib[2] + ADCValues_AfterCalib[3] + ADCValues_AfterCalib[4]);
	  float X = (2 * (ADCValues_AfterCalib[4] - ADCValues_AfterCalib[0]) + (ADCValues_AfterCalib[3] - ADCValues_AfterCalib[1])) * 17 / sum_ADCvalues;
	  return (LINE_COE_1 * X - LINE_COE_2);
}
