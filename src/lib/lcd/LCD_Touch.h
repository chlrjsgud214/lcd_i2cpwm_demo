/*****************************************************************************
 * | File      	:	LCD_Touch.h
 * | Author      :   Waveshare team
 * | Function    :	LCD Touch Pad Driver and Draw
 * | Info        :
 *   Image scanning
 *      Please use progressive scanning to generate images or fonts
 *----------------
 * |	This version:   V1.0
 * | Date        :   2017-08-16
 * | Info        :   Basic version
 *
 ******************************************************************************/
#ifndef __LCD_TOUCH_H_
#define __LCD_TOUCH_H_

#include "DEV_Config.h"
#include "LCD_Driver.h"
#include "LCD_GUI.h"
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/float.h"
#include "fatfs_storage.h"
#include "pca9685.h"
#include <time.h>

#define TP_PRESS_DOWN 0x80
#define TP_PRESSED 0x40

#define STOP 		0
#define INJECT 		1
#define DRAIN	 	2
#define CLEAN 		3
#define OVERFLOW 	4

// Touch screen structure
typedef struct
{
	POINT Xpoint0;
	POINT Ypoint0;
	POINT Xpoint;
	POINT Ypoint;
	uint8_t chStatus;
	uint8_t chType;
	int16_t iXoff;
	int16_t iYoff;
	float fXfac;
	float fYfac;
	// Select the coordinates of the XPT2046 touch
	// screen relative to what scan direction
	LCD_SCAN_DIR TP_Scan_Dir;
} TP_DEV;

// Brush structure
typedef struct
{
	POINT Xpoint;
	POINT Ypoint;
	COLOR Color;
	DOT_PIXEL DotPixel;
} TP_DRAW;

void TP_GetAdFac(void);
void TP_Adjust(void);
void TP_Dialog(void);
void TP_DrawBoard(void);
void TP_Init(LCD_SCAN_DIR Lcd_ScanDir);
void TP_gesmain(void);
void TP_geslog(void);
void TP_gesSDcard(void);
void TP_gesvallist(void);
void TP_gesvalview(void);
void TP_gesmpresultstart(void);
void TP_gessetting(void);
void TP_gessensor(void);
void TP_gessensor_pwm(void);
void TP_gessensor_pwm_bar(void);
void TP_Bmp_view(uint16_t Xpoz, uint16_t Ypoz, const char *Bmpname);
void TP_start_view(uint8_t pagenum);

void TP_Bmp_button(uint16_t Xpoz, uint16_t Ypoz, uint8_t BtnNum);
void TP_Bmp_num(uint16_t Xpoz, uint16_t Ypoz, uint8_t Num, bool back_g);

void Run_page(uint8_t page_num);
#endif
