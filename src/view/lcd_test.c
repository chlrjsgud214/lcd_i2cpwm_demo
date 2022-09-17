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

int lcd_test(void)
{
	//uint8_t counter = 0;
   	// bspInit(); // USB COM Port 활성화 
	System_Init();
	SD_Init();
	DEV_PWM_Init();
	// Driver_Delay_ms(200);
	pca_i2c_init();
	i2c_bus_scan();
	LCD_SCAN_DIR  lcd_scan_dir = SCAN_DIR_DFT;
	LCD_Init(lcd_scan_dir,800);
	TP_Init(lcd_scan_dir);
	

	// GUI_Show();	
	// LCD_SCAN_DIR bmp_scan_dir = SCAN_DIR_DFT;
	// LCD_Show_bmp(bmp_scan_dir,lcd_scan_dir);
	// LCD_SetGramScanWay( 7 ); // BMP용 각도로 변경
	// Storage_OpenReadFile(0, 0, "main.bmp");
	// Storage_OpenReadFile(0, 0, "cat2.bmp");
	// LCD_SetGramScanWay( lcd_scan_dir ); // 터치용 각도로 변경	
	TP_GetAdFac(); // 터치 캘리브레이션
	//TP_Dialog();
	LCD_Clear(LCD_BACKGROUND);
	TP_gesmain();
	// TP_gesvallist();
	//Driver_Delay_ms(50);
	
	while(1){		
		TP_DrawBoard(); 
		//on_uart_rx();
		//printf("test\r\n");
	}
	return 0;
}

