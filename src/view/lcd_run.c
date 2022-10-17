/*****************************************************************************
* | File      	:   lcd_2in8.c
* | Author      :   Waveshare team
* | Function    :   2.9inch e-paper V2 test demo
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-01-20
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#include "main.h"
#include "LCD_Driver.h"
#include "LCD_Touch.h"
#include "LCD_Bmp.h"
#include "hardware/watchdog.h"
#include "fatfs_storage.h"
#include "pca9685.h"

int lcd_run(void)
{
	System_Init();							  // USB Serial , SPI , default GPIO set
	SENSOR_GPIO_Init();						  // 포토센서 In pin GPIO set
	SD_Init();								  // SD Pin set , SD card 확인
	pca_i2c_init();							  // i2c PWM 16ch 확인
	i2c_bus_scan();							  // PCA addr 확인
	LCD_SCAN_DIR lcd_scan_dir = SCAN_DIR_DFT; // 화면 방향
	LCD_Init(lcd_scan_dir, 800);			  // LCD 활성화 및 기본설정 SET
	TP_Init(lcd_scan_dir);					  // 터치 활성화

	TP_GetAdFac(); // 터치 캘리브레이션
	LCD_Clear(LCD_BACKGROUND);

	TP_gesmain();					  // 메인화면 표현
	TP_Bmp_view(0, 0, "b_str.bmp");	  // 버튼 백그라운드 저장 1~8
	TP_Bmp_view(0, 0, "num1216.bmp"); // 숫자 백그라운드 저장 0~99
	// show_button(12,54,5);
	// for (int i = 0; i < 10; i++)
	// {
	// 	for (int j = 0; j < 10; j++)
	// 	{

	// 		TP_Bmp_num(i * 25, j*20, i*j);
	// 	}
	// }
	while (1)
	{
		TP_DrawBoard(); // 터치 감지시 동작 이외엔 대기
	}
	return 0;
}
