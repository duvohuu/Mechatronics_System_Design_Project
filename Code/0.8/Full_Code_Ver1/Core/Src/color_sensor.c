/*
 * color_sensor.c
 *
 *  Created on: Aug 25, 2025
 *      Author: Asus
 */

#include "color_sensor.h"
#include "others.h"
#include "params.h"

extern I2C_HandleTypeDef hi2c1;
uint8_t _tcs34725Initialised = 0;
int RED = 0, GREEN = 0, BLUE = 0;



/* Writes a register and an 8 bit value over I2C */
void write8 (uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 2, 100);
}

/* Reads an 8 bit value over I2C */
uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

/* Reads a 16 bit values over I2C */
uint16_t read16(uint8_t reg)
{
	uint16_t ret;
    uint8_t txBuffer[1],rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
    return ret;
}

void enable(void)
{
	  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
	  HAL_Delay(3);
	  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
	  HAL_Delay(50);
}

void disable(void)
{
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
  /* Make sure we're actually connected */
  uint8_t readValue = read8(TCS34725_ID);
  if ((readValue != 0x44) && (readValue != 0x10)&& (readValue != 0x4d))
  {
    return;
  }
  _tcs34725Initialised = 1;
  /* Set default integration time and gain */
  setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  setGain(TCS34725_GAIN_4X);
  /* Note: by default, the device is in power down mode on bootup */
  enable();
}

/* Get raw data */
void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (_tcs34725Initialised == 0) tcs3272_init();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);

  /* Delay time is from page no 16/26 from the datasheet  (256 − ATIME)* 2.4ms */
  HAL_Delay(50); // Set delay for (256 − 0xEB)* 2.4ms = 50ms
}



/* Get Red, Green and Blue color from Raw Data */
void getRGB(int *RED, int *GREEN, int *BLUE)
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);
    if(rawClear == 0)
    {
      *RED = 0;
      *GREEN = 0;
      *BLUE = 0;
    }
    else
    {
      *RED = (int)rawRed * 255 / rawClear;
      *GREEN = (int)rawGreen * 255 / rawClear;
      *BLUE = (int)rawBlue * 255 / rawClear;
    }
}


void detect_color(void)
{
	getRGB(&RED, &GREEN, &BLUE);
    // Red
    if ((RED > 80 && RED < 100) &&
        (BLUE > 65 && BLUE < 85) &&
        (GREEN > 80 && GREEN < 90))
    {
        red_package = true;
		other_package = false;
		blue_package = false;
	    HAL_Delay(5000);
	    empty = false;
	    stop = false;
	    has_stop_for_package = false;
	    is_delivery_area = false;
    }

    // Blue
    else if ((RED > 10 && RED < 45) &&
             (BLUE > 105 && BLUE < 130) &&
             (GREEN > 80 && GREEN < 100))
    {
		blue_package = true;
		red_package = false;
		other_package = false;
	    HAL_Delay(5000);
	    empty = false;
	    stop = false;
	    has_stop_for_package = false;
	    is_delivery_area = false;
    }
    // None
    else {
        blue_package = false;
		red_package = false;
		other_package = true;
	    empty = true;
		stop = true;
		has_stop_for_package = true;
//		is_delivery_area = true;
    }
}
