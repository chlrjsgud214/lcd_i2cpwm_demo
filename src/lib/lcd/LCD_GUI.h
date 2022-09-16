/*****************************************************************************
* | File      	:	LCD_GUI.h
* | Author      :   Waveshare team
* | Function    :	Achieve drawing: draw points, lines, boxes, circles and
*                   their size, solid dotted line, solid rectangle hollow
*					rectangle, solid circle hollow circle.
* | Info        :
*   Achieve display characters: Display a single character, string, number
*   Achieve time display: adaptive size display time minutes and seconds
*----------------
* |	This version:   V1.0
* | Date        :   2017-08-16
* | Info        :   Basic version
*
******************************************************************************/

/****************************Upper application layer**************************/
#ifndef __LCD_GUI_H
#define __LCD_GUI_H

#include "LCD_Driver.h"
#include "fonts.h"


#define LOW_Speed_Show 0
#define HIGH_Speed_Show 1

typedef enum
{
  LCD_FONT_07x10,
  LCD_FONT_11x18,
  LCD_FONT_16x26,
  LCD_FONT_HAN,
  LCD_FONT_MAX
} LcdFont;

typedef enum
{
  LCD_ALIGN_H_LEFT    = (1<<0),
  LCD_ALIGN_H_CENTER  = (1<<1),
  LCD_ALIGN_H_RIGHT   = (1<<2),
  LCD_ALIGN_V_TOP     = (1<<3),
  LCD_ALIGN_V_CENTER  = (1<<4),
  LCD_ALIGN_V_BOTTOM  = (1<<5),
} LcdStringAlign;

typedef enum
{
  LCD_RESIZE_NEAREST,
  LCD_RESIZE_BILINEAR
} LcdResizeMode;

typedef struct lcd_driver_t_ lcd_driver_t;

typedef struct lcd_driver_t_
{
  bool     (*init)(void);
  bool     (*reset)(void);
  void     (*setWindow)(int32_t x, int32_t y, int32_t w, int32_t h);
  uint16_t (*getWidth)(void);
  uint16_t (*getHeight)(void);
  bool     (*setCallBack)(void (*p_func)(void));
  bool     (*sendBuffer)(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);

} lcd_driver_t;
/********************************************************************************
function:
			dot pixel
********************************************************************************/
typedef enum {
    DOT_PIXEL_1X1  = 1,		// dot pixel 1 x 1
    DOT_PIXEL_2X2  , 		// dot pixel 2 X 2
    DOT_PIXEL_3X3  ,		// dot pixel 3 X 3
    DOT_PIXEL_4X4  ,		// dot pixel 4 X 4
    DOT_PIXEL_5X5  , 		// dot pixel 5 X 5
    DOT_PIXEL_6X6  , 		// dot pixel 6 X 6
    DOT_PIXEL_7X7  , 		// dot pixel 7 X 7
    DOT_PIXEL_8X8  , 		// dot pixel 8 X 8
} DOT_PIXEL;
#define DOT_PIXEL_DFT  DOT_PIXEL_1X1  //Default dot pilex

/********************************************************************************
function:
			dot Fill style
********************************************************************************/
typedef enum {
    DOT_FILL_AROUND  = 1,		// dot pixel 1 x 1
    DOT_FILL_RIGHTUP  , 		// dot pixel 2 X 2
} DOT_STYLE;
#define DOT_STYLE_DFT  DOT_FILL_AROUND  //Default dot pilex
/********************************************************************************
function:
			solid line and dotted line
********************************************************************************/
typedef enum {
    LINE_SOLID = 0,
    LINE_DOTTED,
} LINE_STYLE;

/********************************************************************************
function:
			DRAW Internal fill
********************************************************************************/
typedef enum {
    DRAW_EMPTY = 0,
    DRAW_FULL,
} DRAW_FILL;

/********************************************************************************
function:
	time
********************************************************************************/
typedef struct {
    uint16_t Year;  //0000
    uint8_t  Month; //1 - 12
    uint8_t  Day;   //1 - 30
    uint8_t  Hour;  //0 - 23
    uint8_t  Min;   //0 - 59
    uint8_t  Sec;   //0 - 59
} DEV_TIME;
extern DEV_TIME sDev_time;

enum class_color {
white   =  0xFFFF,
black   =  0x0000,
blue    =  0x001F,
BRED    =  0XF81F,
GRED 	  =  0XFFE0,
GBLUE	  =  0X07FF,
red     =  0xF800,
MAGENTA =  0xF81F,
green   =  0x07E0,
CYAN    =  0x7FFF,
yellow  =  0xFFE0,
BROWN  	=	 0XBC40,
BRRED 	=	 0XFC07,
GRAY  	=	 0X8430,

};
/********************************************************************************
function:
			Defines commonly used colors for the display
********************************************************************************/
#define LCD_BACKGROUND		WHITE   //Default background color
#define FONT_BACKGROUND		WHITE   //Default font background color
#define FONT_FOREGROUND	    GRED    //Default font foreground color

#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED 		   0XFFE0
#define GBLUE		   0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN 		   0XBC40
#define BRRED 		   0XFC07
#define GRAY  		   0X8430

/********************************************************************************
function:
			Macro definition variable name
********************************************************************************/
//Clear
void GUI_Clear(COLOR Color);

//Drawing
void GUI_DrawPoint(POINT Xpoint, POINT Ypoint, COLOR Color, DOT_PIXEL Dot_Pixel, DOT_STYLE Dot_FillWay);
void GUI_DrawLine(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, LINE_STYLE Line_Style, DOT_PIXEL Dot_Pixel);
void GUI_DrawRectangle(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, COLOR Color, DRAW_FILL Filled , DOT_PIXEL Dot_Pixel );
void GUI_DrawCircle(POINT X_Center, POINT Y_Center, LENGTH Radius, COLOR Color, DRAW_FILL Draw_Fill , DOT_PIXEL Dot_Pixel );

//pic
void GUI_Disbitmap(POINT Xpoint, POINT Ypoint, const unsigned char *pMap, POINT Width, POINT Height);
void GUI_DisGrayMap(POINT Xpoint, POINT Ypoint, const unsigned char *pBmp);

//Display string
void GUI_DisChar(POINT Xstart, POINT Ystart, const char Acsii_Char, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground);
void GUI_DisString_EN(POINT Xstart, POINT Ystart, const char * pString, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
void GUI_DisNum(POINT Xpoint, POINT Ypoint, int32_t Nummber, sFONT* Font, COLOR Color_Background, COLOR Color_Foreground );
void GUI_Showtime(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend, DEV_TIME *pTime, COLOR Color, COLOR FColor);
//show
void GUI_Show(void); 

bool lcdDrawAvailable(void);
bool lcdRequestDraw(void);
void lcdUpdateDraw(void);
void lcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code);
void lcdPrintf(int x, int y, uint16_t color,  const char *fmt, ...);
void lcdSetFont(LcdFont font);
LcdFont lcdGetFont(void);
void lcdPrintfResize(int x, int y, uint16_t color,  float ratio_h, const char *fmt, ...);
//void lcdSetResizeMode(LcdResizeMode mode);

#endif





