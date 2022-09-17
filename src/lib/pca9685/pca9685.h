/*************************************************************************
 * pca9685.h
 *
 * This software is a devLib extension to wiringPi <http://wiringpi.com/>
 * and enables it to control the Adafruit PCA9685 16-Channel 12-bit
 * PWM/Servo Driver <http://www.adafruit.com/products/815> via I2C interface.
 *
 * Copyright (c) 2014 Reinhard Sprung
 *
 * If you have questions or improvements email me at
 * reinhard.sprung[at]gmail.com
 *
 * This software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The given code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You can view the contents of the licence at <http://www.gnu.org/licenses/>.
 **************************************************************************
 */
 

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
 


#define PICO_i2c1 i2c1
#define PICO_I2C1_SDA_PIN 26
#define PICO_I2C1_SCL_PIN 27

#define PWM_ON_L    0x06
#define PWM_ON_H    0x07
#define PWM_OFF_L   0x08
#define PWM_OFF_H   0x09

#define MULTI 4	// For the other 15 channels
#define ALLLED_ON_L 0xFA    //load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALLLED_ON_H 0xFB	//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALLLED_OFF_L 0xFC	//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALLLED_OFF_H 0xFD	//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALE 0xFE		//prescaler for output frequency
#define CLOCK_FREQ 250000000.0 //250MHz default osc clock


int pca_i2c_init();
void i2c_bus_scan();
void setPWM(uint8_t led,int on_value);
void setPWMFreq(int freq);
void read_PCA_reg(uint8_t reg,uint8_t len);
void write_PCA_reg(uint8_t reg,uint8_t rx);
void getPCAmode(uint8_t setPCA);
// // Setup a pca9685 at the specific i2c address
// extern int pca9685Setup(const int pinBase, const int i2cAddress/* = 0x40*/, float freq/* = 50*/);

// // You now have access to the following wiringPi functions:
// //
// // void pwmWrite (int pin, int value)
// //		if value <= 0, set full-off
// //		else if value >= 4096, set full-on
// //		else set PWM
// //
// // void digitalWrite (int pin, int value)
// // 		if value != 0, set full-on
// //		else set full-off
// //
// // int digitalRead (int pin)
// //		read off-register
// //		To get PWM: mask with 0xFFF
// //		To get full-off bit: mask with 0x1000
// //		Note: ALL_LED pin will always return 0
// //
// // int analogRead (int pin)
// //		read on-register
// //		To get PWM: mask with 0xFFF
// //		To get full-on bit: mask with 0x1000
// //		Note: ALL_LED pin will always return 0



// // Advanced controls
// // You can use the file descriptor returned from the setup function to access the following features directly on each connected pca9685
// extern void pca9685PWMFreq(int fd, float freq);
// extern void pca9685PWMReset(int fd);
// extern void pca9685PWMWrite(int fd, int pin, int on, int off);
// extern void pca9685PWMRead(int fd, int pin, int *on, int *off);

// extern void pca9685FullOn(int fd, int pin, int tf);
// extern void pca9685FullOff(int fd, int pin, int tf);

// #ifdef __cplusplus
// }
// #endif
