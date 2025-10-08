/*
 * color_sensor.h
 *
 *  Created on: Aug 25, 2025
 *      Author: Asus
 */

#include <stdint.h>
//#include "stm32f1xx_hal_i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#ifndef INC_COLOR_SENSOR_H_
#define INC_COLOR_SENSOR_H_


#define TCS34725_ADDRESS          (0x29 << 1) /* I2C address */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)
/* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
/* set the correct delay time in void getRawData() for TCS34725_INTEGRATIONTIME */
#define TCS34725_INTEGRATIONTIME_50MS   0xEB  /* 50ms  - 20 cycles */
#define TCS34725_GAIN_4X                0x01  /* 4x gain  */
extern uint8_t _tcs34725Initialised;

extern int RED, GREEN, BLUE;



void write8 (uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
void enable(void);
void disable(void);
void setIntegrationTime(uint8_t it);
void setGain(uint8_t gain);
void tcs3272_init(void);
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(int *RED, int *GREEN, int *BLUE);
/* FUNCTIONS FOR DETECT COLOR */
void detect_color(void);


#endif /* INC_COLOR_SENSOR_H_ */
