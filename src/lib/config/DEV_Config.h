/*****************************************************************************
* | File      	:	DEV_Config.c
* | Author      :   Waveshare team
* | Function    :	GPIO Function
* | Info        :
*   Provide the hardware underlying interface	 
*----------------
* |	This version:   V1.0
* | Date        :   2018-01-11
* | Info        :   Basic version
*
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_


#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "stdio.h"
#include "pico/time.h"
#include "hardware/rtc.h"

#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

#define LCD_RST_PIN		15	
#define LCD_DC_PIN		6 // GP8 -> GP6
#define LCD_CS_PIN		7 // GP9 -> GP7
#define LCD_CLK_PIN		10
#define LCD_BKL_PIN		13
#define LCD_MOSI_PIN	11
#define LCD_MISO_PIN	12
#define TP_CS_PIN		16
#define TP_IRQ_PIN		17
#define SD_CS_PIN		22
#define PWM_LED         25
#define PWM_CH0         0
#define PWM_CH1         1
#define PWM_CH2         2

#define SENSOR_GP0      0
#define SENSOR_GP1      1
#define SENSOR_GP2      2
#define SENSOR_GP3      3
#define SENSOR_GP4      4
#define S0       gpio_get(SENSOR_GP0)
#define S1       gpio_get(SENSOR_GP1)
#define S2       gpio_get(SENSOR_GP2)
#define S3       gpio_get(SENSOR_GP3)
#define S4       gpio_get(SENSOR_GP4)

#define SPI_PORT		spi1
#define  MAX_BMP_FILES  25 

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#ifndef cmax
#define cmax(a,b) (((a) > (b)) ? (a) : (b))
#define cmin(a,b) (((a) < (b)) ? (a) : (b))
#endif


#ifndef cmap
#define cmap(value, in_min, in_max, out_min, out_max) ((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
#endif







/*------------------------------------------------------------------------------------------------------*/
 
void DEV_Digital_Write(UWORD Pin, UBYTE Value);
UBYTE DEV_Digital_Read(UWORD Pin);
void DEV_GPIO_Mode(UWORD Pin, UWORD Mode);
void DEV_GPIO_Init(void);

uint8_t System_Init(void);
void System_Exit(void);
uint8_t SPI4W_Write_Byte(uint8_t value);
uint8_t SPI4W_Read_Byte(uint8_t value);

void Driver_Delay_ms(uint32_t xms);
void Driver_Delay_us(uint32_t xus);

void DEV_PWM_Init(void);
void PWMON(uint16_t val);
void PWMOFF(void);
#endif
