/*****************************************************************************
* | File      	:	LCD_Touch.c
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
#include "LCD_Touch.h"

static uint32_t pwmgui=0;
extern LCD_DIS sLCD_DIS;
extern uint8_t id;
static TP_DEV sTP_DEV;
static TP_DRAW sTP_Draw;
static DEV_TIME settime;
bool main_status =true;
uint16_t pagestatus=0;
bool pwmstatus=false;
bool ox_sensing=false;
bool sstart=false;
bool sstop=false;
uint16_t s_count=0;
uint8_t pwmval=0;
uint32_t pwmout=0;
uint8_t start_status=0;
uint8_t i2c_writebuf[3];

const char *ox_str="";
const char * ox_ch[10]={0x00,};
const char * kitno[]={"001","002","003","004"};
const char * reml[]={"400ml","3000ml","500ml","1000ml"};
const char * reppm[]={"25ppm","100ppm","100EA","1000ppm"};
const char * reday[]={"22.08.18","22.08.08","22.07.28","22.07.08"};    
const char * retime[]={"12:10","16:22","07:45","00:00"};    
int ycur=35;


bool ret = false;

/*******************************************************************************
function:
		Read the ADC of the channel
parameter:
	Channel_Cmd :	0x90: Read channel Y +, select the ADC resolution is 12 bits, set to differential mode
					0xd0: Read channel x +, select the ADC resolution is 12 bits, set to differential mode
*******************************************************************************/
static uint16_t TP_Read_ADC(uint8_t CMD)
{
    uint16_t Data = 0;

    //A cycle of at least 400ns.
    DEV_Digital_Write(TP_CS_PIN,0);

    SPI4W_Write_Byte(CMD);
    Driver_Delay_us(200);

    //	dont write 0xff, it will block xpt2046  
    //Data = SPI4W_Read_Byte(0Xff);
    Data = SPI4W_Read_Byte(0X00);
    Data <<= 8;//7bit
    Data |= SPI4W_Read_Byte(0X00);
    //Data = SPI4W_Read_Byte(0Xff);
    Data >>= 3;//5bit
    DEV_Digital_Write(TP_CS_PIN,1);
    return Data;
}

/*******************************************************************************
function:
		Read the 5th channel value and exclude the maximum and minimum returns the average
parameter:
	Channel_Cmd :	0x90 :Read channel Y +
					0xd0 :Read channel x +
*******************************************************************************/
#define READ_TIMES  5	//Number of readings
#define LOST_NUM    1	//Discard value
static uint16_t TP_Read_ADC_Average(uint8_t Channel_Cmd)
{
    uint8_t i, j;
    uint16_t Read_Buff[READ_TIMES];
    uint16_t Read_Sum = 0, Read_Temp = 0;
    //LCD SPI speed = 3 MHz
    spi_set_baudrate(SPI_PORT,3000000);
    //Read and save multiple samples
    for(i = 0; i < READ_TIMES; i++){
		Read_Buff[i] = TP_Read_ADC(Channel_Cmd);
		Driver_Delay_us(200);
	}
    //LCD SPI speed = 18 MHz
    spi_set_baudrate(SPI_PORT,18000000);
    //Sort from small to large
    for (i = 0; i < READ_TIMES  -  1; i ++) {
        for (j = i + 1; j < READ_TIMES; j ++) {
            if (Read_Buff[i] > Read_Buff[j]) {
                Read_Temp = Read_Buff[i];
                Read_Buff[i] = Read_Buff[j];
                Read_Buff[j] = Read_Temp;
            }
        }
    }

    //Exclude the largest and the smallest
    for (i = LOST_NUM; i < READ_TIMES - LOST_NUM; i ++)
        Read_Sum += Read_Buff[i];

    //Averaging
    Read_Temp = Read_Sum / (READ_TIMES - 2 * LOST_NUM);

    return Read_Temp;
}

/*******************************************************************************
function:
		Read X channel and Y channel AD value
parameter:
	Channel_Cmd :	0x90 :Read channel Y +
					0xd0 :Read channel x +
*******************************************************************************/
static void TP_Read_ADC_XY(uint16_t *pXCh_Adc, uint16_t  *pYCh_Adc )
{
    *pXCh_Adc = TP_Read_ADC_Average(0xD0);
    *pYCh_Adc = TP_Read_ADC_Average(0x90);
}

/*******************************************************************************
function:
		2 times to read the touch screen IC, and the two can not exceed the deviation,
		ERR_RANGE, meet the conditions, then that the correct reading, otherwise the reading error.
parameter:
	Channel_Cmd :	pYCh_Adc = 0x90 :Read channel Y +
					pXCh_Adc = 0xd0 :Read channel x +
*******************************************************************************/
#define ERR_RANGE 50	//tolerance scope
static bool TP_Read_TwiceADC(uint16_t *pXCh_Adc, uint16_t  *pYCh_Adc )
{
    uint16_t XCh_Adc1, YCh_Adc1, XCh_Adc2, YCh_Adc2;

    //Read the ADC values Read the ADC values twice
    TP_Read_ADC_XY(&XCh_Adc1, &YCh_Adc1);
	Driver_Delay_us(10);
    TP_Read_ADC_XY(&XCh_Adc2, &YCh_Adc2);
	Driver_Delay_us(10);
	
    //The ADC error used twice is greater than ERR_RANGE to take the average
    if( ((XCh_Adc2 <= XCh_Adc1 && XCh_Adc1 < XCh_Adc2 + ERR_RANGE) ||
         (XCh_Adc1 <= XCh_Adc2 && XCh_Adc2 < XCh_Adc1 + ERR_RANGE))
        && ((YCh_Adc2 <= YCh_Adc1 && YCh_Adc1 < YCh_Adc2 + ERR_RANGE) ||
            (YCh_Adc1 <= YCh_Adc2 && YCh_Adc2 < YCh_Adc1 + ERR_RANGE))) {
        *pXCh_Adc = (XCh_Adc1 + XCh_Adc2) / 2;
        *pYCh_Adc = (YCh_Adc1 + YCh_Adc2) / 2;
        return true;
    }

    //The ADC error used twice is less than ERR_RANGE returns failed
    return false;
}

/*******************************************************************************
function:
		Calculation
parameter:
		chCoordType:
					1 : calibration
					0 : relative position
*******************************************************************************/
static uint8_t TP_Scan(uint8_t chCoordType)
{
    //In X, Y coordinate measurement, IRQ is disabled and output is low
    if (!DEV_Digital_Read(TP_IRQ_PIN)) {//Press the button to press
        //Read the physical coordinates
        if (chCoordType) {
            TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint);
            //Read the screen coordinates
        } else if (TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint)) {

			if(LCD_2_8==id){ // 시작 좌표 설정
				sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Ypoint +
									  sTP_DEV.iXoff;
					sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Xpoint +
									  sTP_DEV.iYoff;
			}else{
				//DEBUG("(Xad,Yad) = %d,%d\r\n",sTP_DEV.Xpoint,sTP_DEV.Ypoint);
				if(sTP_DEV.TP_Scan_Dir == R2L_D2U) {		//Converts the result to screen coordinates
					sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Xpoint +
									  sTP_DEV.iXoff;
					sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Ypoint +
									  sTP_DEV.iYoff;
				} else if(sTP_DEV.TP_Scan_Dir == L2R_U2D) {
					sTP_Draw.Xpoint = sLCD_DIS.LCD_Dis_Column -
									  sTP_DEV.fXfac * sTP_DEV.Xpoint -
									  sTP_DEV.iXoff;
					sTP_Draw.Ypoint = sLCD_DIS.LCD_Dis_Page -
									  sTP_DEV.fYfac * sTP_DEV.Ypoint -
									  sTP_DEV.iYoff;
				} else if(sTP_DEV.TP_Scan_Dir == U2D_R2L) {
					sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Ypoint +
									  sTP_DEV.iXoff;
					sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Xpoint +
									  sTP_DEV.iYoff;
				} else {
					sTP_Draw.Xpoint = sLCD_DIS.LCD_Dis_Column -
									  sTP_DEV.fXfac * sTP_DEV.Ypoint -
									  sTP_DEV.iXoff;
					sTP_Draw.Ypoint = sLCD_DIS.LCD_Dis_Page -
									  sTP_DEV.fYfac * sTP_DEV.Xpoint -
									  sTP_DEV.iYoff;
				}
				 ///DEBUG("( x , y ) = %d,%d\r\n",sTP_Draw.Xpoint,sTP_Draw.Ypoint);
			}
		}
        if (0 == (sTP_DEV.chStatus & TP_PRESS_DOWN)) {	//Not being pressed
            sTP_DEV.chStatus = TP_PRESS_DOWN | TP_PRESSED;
            sTP_DEV.Xpoint0 = sTP_DEV.Xpoint;
            sTP_DEV.Ypoint0 = sTP_DEV.Ypoint;
        }
    } else {
        if (sTP_DEV.chStatus & TP_PRESS_DOWN) {	//0x80
            sTP_DEV.chStatus &= ~(1 << 7);		//0x00
        } else {
            sTP_DEV.Xpoint0 = 0;
            sTP_DEV.Ypoint0 = 0;
            sTP_DEV.Xpoint = 0xffff;
            sTP_DEV.Ypoint = 0xffff;
        }
    }

    return (sTP_DEV.chStatus & TP_PRESS_DOWN);
}

/*******************************************************************************
function:
		Draw Cross
parameter:
			Xpoint :	The x coordinate of the point
			Ypoint :	The y coordinate of the point
			Color  :	Set color
*******************************************************************************/
static void TP_DrawCross(POINT Xpoint, POINT Ypoint, COLOR Color)
{	
    GUI_DrawLine(Xpoint  -  12, Ypoint, Xpoint + 12, Ypoint,
                 Color, LINE_SOLID, DOT_PIXEL_1X1);
    GUI_DrawLine(Xpoint, Ypoint  -  12, Xpoint, Ypoint + 12,
                 Color, LINE_SOLID, DOT_PIXEL_1X1);
    GUI_DrawPoint(Xpoint, Ypoint, Color, DOT_PIXEL_2X2 , DOT_FILL_AROUND);
    GUI_DrawCircle(Xpoint, Ypoint, 6, Color, DRAW_EMPTY, DOT_PIXEL_1X1);
}


/*******************************************************************************
function:
		The corresponding ADC value is displayed on the LC
parameter:
			(Xpoint0 ,Xpoint0):	The coordinates of the first point
			(Xpoint1 ,Xpoint1):	The coordinates of the second point
			(Xpoint2 ,Xpoint2):	The coordinates of the third point
			(Xpoint3 ,Xpoint3):	The coordinates of the fourth point
			hwFac	:	Percentage of error
*******************************************************************************/
static void TP_ShowInfo(POINT Xpoint0, POINT Ypoint0,
                        POINT Xpoint1, POINT Ypoint1,
                        POINT Xpoint2, POINT Ypoint2,
                        POINT Xpoint3, POINT Ypoint3,
                        POINT hwFac)
{
	if(LCD_2_8==id){
		
	}else{
		sFONT* TP_Font = &Font16;
		LENGTH TP_Dx = 	TP_Font->Width;

		GUI_DrawRectangle(40, 160, 250, 270, WHITE, DRAW_FULL, DOT_PIXEL_1X1);

		GUI_DisString_EN(40, 160, "x1", TP_Font, FONT_BACKGROUND, RED);
		GUI_DisString_EN(40 + 100, 160, "y1", TP_Font, FONT_BACKGROUND, RED);

		GUI_DisString_EN(40, 180, "x2", TP_Font, FONT_BACKGROUND, RED);
		GUI_DisString_EN(40 + 100, 180, "y2", TP_Font, FONT_BACKGROUND, RED);

		GUI_DisString_EN(40, 200, "x3", TP_Font, FONT_BACKGROUND, RED);
		GUI_DisString_EN(40 + 100, 200, "y3", TP_Font, FONT_BACKGROUND, RED);

		GUI_DisString_EN(40, 220, "x4", TP_Font, FONT_BACKGROUND, RED);
		GUI_DisString_EN(40 + 100, 220, "y4", TP_Font, FONT_BACKGROUND, RED);

		GUI_DisString_EN(40, 240, "fac is : ", TP_Font, FONT_BACKGROUND, RED);

		GUI_DisNum(40 + 3 * TP_Dx, 160, Xpoint0, TP_Font, FONT_BACKGROUND, RED);
		GUI_DisNum(40 + 3 * TP_Dx + 100, 160, Ypoint0, TP_Font, FONT_BACKGROUND, RED);

		GUI_DisNum(40 + 3 * TP_Dx, 180, Xpoint1, TP_Font, FONT_BACKGROUND, RED);
		GUI_DisNum(40 + 3 * TP_Dx + 100, 180, Ypoint1, TP_Font, FONT_BACKGROUND, RED);

		GUI_DisNum(40 + 3 * TP_Dx, 200, Xpoint2, TP_Font, FONT_BACKGROUND, RED);
		GUI_DisNum(40 + 3 * TP_Dx + 100, 200, Ypoint2, TP_Font, FONT_BACKGROUND, RED);

		GUI_DisNum(40 + 3 * TP_Dx, 220, Xpoint3, TP_Font, FONT_BACKGROUND, RED);
		GUI_DisNum(40 + 3 * TP_Dx + 100, 220, Ypoint3, TP_Font, FONT_BACKGROUND, RED);

		GUI_DisNum(40 + 10 * TP_Dx , 240, hwFac, TP_Font, FONT_BACKGROUND, RED);
	}
}

/*******************************************************************************
function:
		Touch screen adjust
*******************************************************************************/
void TP_Adjust(void)
{
    uint8_t  cnt = 0;
    uint16_t XYpoint_Arr[4][2];
    uint32_t Dx, Dy;
    uint16_t Sqrt1, Sqrt2;
    float Dsqrt;

    LCD_Clear(LCD_BACKGROUND);
    GUI_DisString_EN(0, 60, "Please use the stylus click the cross"\
                     "on the screen. The cross will always move until"\
                     "the screen adjustment is completed.",
                     &Font16, FONT_BACKGROUND, RED);

    uint8_t Mar_Val = 12;
    TP_DrawCross(Mar_Val, Mar_Val, RED);

    sTP_DEV.chStatus = 0;
    while (1) {
        TP_Scan(1);
        if((sTP_DEV.chStatus & 0xC0) == TP_PRESSED) {
            sTP_DEV.chStatus &= ~(1 << 6);
            XYpoint_Arr[cnt][0] = sTP_DEV.Xpoint;
            XYpoint_Arr[cnt][1] = sTP_DEV.Ypoint;
			printf("X%d,Y%d = %d,%d\r\n",cnt,cnt,XYpoint_Arr[cnt][0],XYpoint_Arr[cnt][1]);
            cnt ++;
            Driver_Delay_ms(200);

            switch(cnt) {
            case 1:
                //DEBUG("not touch TP_IRQ 2 = %d\r\n", GET_TP_IRQ);
                TP_DrawCross(Mar_Val, Mar_Val, WHITE);
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val, Mar_Val, RED);
                Driver_Delay_ms(200);
                break;
            case 2:
                //DEBUG("not touch TP_IRQ 3 = %d\r\n", GET_TP_IRQ);
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val, Mar_Val, WHITE);
                TP_DrawCross(Mar_Val, sLCD_DIS.LCD_Dis_Page - Mar_Val, RED);
                Driver_Delay_ms(200);
                break;
            case 3:
                //DEBUG("not touch TP_IRQ 4 = %d\r\n", GET_TP_IRQ);
                TP_DrawCross(Mar_Val, sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                             sLCD_DIS.LCD_Dis_Page - Mar_Val, RED);
                Driver_Delay_ms(200);
                break;
            case 4:

                // 1.Compare the X direction
                Dx = abs((int16_t)(XYpoint_Arr[0][0] -
                                  XYpoint_Arr[1][0]));//x1 - x2
                Dy = abs((int16_t)(XYpoint_Arr[0][1] -
                                  XYpoint_Arr[1][1]));//y1 - y2
                Dx *= Dx;
                Dy *= Dy;
                Sqrt1 = sqrt(Dx + Dy);

                Dx = abs((int16_t)(XYpoint_Arr[2][0] -
                                  XYpoint_Arr[3][0]));//x3 - x4
                Dy = abs((int16_t)(XYpoint_Arr[2][1] -
                                  XYpoint_Arr[3][1]));//y3 - y4
                Dx *= Dx;
                Dy *= Dy;
                Sqrt2 = sqrt(Dx + Dy);

                Dsqrt = (float)Sqrt1 / Sqrt2;
                if(Dsqrt < 0.95 || Dsqrt > 1.05 || Sqrt1 == 0 || Sqrt2 == 0) {
                   //DEBUG("Adjust X direction \r\n");
                   cnt = 0;
                   TP_ShowInfo(XYpoint_Arr[0][0], XYpoint_Arr[0][1],
                               XYpoint_Arr[1][0], XYpoint_Arr[1][1],
                               XYpoint_Arr[2][0], XYpoint_Arr[2][1],
                               XYpoint_Arr[3][0], XYpoint_Arr[3][1],
                               Dsqrt * 100);
                   Driver_Delay_ms(1000);
                   TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                                sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                   TP_DrawCross(Mar_Val, Mar_Val, RED);
                   continue;
                }

                // 2.Compare the Y direction
                Dx = abs((int16_t)(XYpoint_Arr[0][0] -
                                  XYpoint_Arr[2][0]));//x1 - x3
                Dy = abs((int16_t)(XYpoint_Arr[0][1] -
                                  XYpoint_Arr[2][1]));//y1 - y3
                Dx *= Dx;
                Dy *= Dy;
                Sqrt1 = sqrt(Dx + Dy);

                Dx = abs((int16_t)(XYpoint_Arr[1][0] -
                                  XYpoint_Arr[3][0]));//x2 - x4
                Dy = abs((int16_t)(XYpoint_Arr[1][1] -
                                  XYpoint_Arr[3][1]));//y2 - y4
                Dx *= Dx;
                Dy *= Dy;
                Sqrt2 = sqrt(Dx + Dy);//

                Dsqrt = (float)Sqrt1 / Sqrt2;
                if(Dsqrt < 0.95 || Dsqrt > 1.05) {
                   //DEBUG("Adjust Y direction \r\n");
                   cnt = 0;
                   TP_ShowInfo(XYpoint_Arr[0][0], XYpoint_Arr[0][1],
                               XYpoint_Arr[1][0], XYpoint_Arr[1][1],
                               XYpoint_Arr[2][0], XYpoint_Arr[2][1],
                               XYpoint_Arr[3][0], XYpoint_Arr[3][1],
                               Dsqrt * 100);
                   Driver_Delay_ms(1000);
                   TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                                sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                   TP_DrawCross(Mar_Val, Mar_Val, RED);
                   continue;
                }//

                //3.Compare diagonal
                Dx = abs((int16_t)(XYpoint_Arr[1][0] -
                                  XYpoint_Arr[2][0]));//x1 - x3
                Dy = abs((int16_t)(XYpoint_Arr[1][1] -
                                  XYpoint_Arr[2][1]));//y1 - y3
                Dx *= Dx;
                Dy *= Dy;
                Sqrt1 = sqrt(Dx + Dy);//;

                Dx = abs((int16_t)(XYpoint_Arr[0][0] -
                                  XYpoint_Arr[3][0]));//x2 - x4
                Dy = abs((int16_t)(XYpoint_Arr[0][1] -
                                  XYpoint_Arr[3][1]));//y2 - y4
                Dx *= Dx;
                Dy *= Dy;
                Sqrt2 = sqrt(Dx + Dy);//

                Dsqrt = (float)Sqrt1 / Sqrt2;
                if(Dsqrt < 0.95 || Dsqrt > 1.05) {
                   printf("Adjust diagonal direction\r\n");
                   cnt = 0;
                   TP_ShowInfo(XYpoint_Arr[0][0], XYpoint_Arr[0][1],
                               XYpoint_Arr[1][0], XYpoint_Arr[1][1],
                               XYpoint_Arr[2][0], XYpoint_Arr[2][1],
                               XYpoint_Arr[3][0], XYpoint_Arr[3][1],
                               Dsqrt * 100);
                   Driver_Delay_ms(1000);
                   TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                                sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                   TP_DrawCross(Mar_Val, Mar_Val, RED);
                   continue;
                }

                //4.Get the scale factor and offset
                //Get the scanning direction of the touch screen
                sTP_DEV.TP_Scan_Dir = sLCD_DIS.LCD_Scan_Dir;
                sTP_DEV.fXfac = 0;

                //According to the display direction to get
                //the corresponding scale factor and offset
                if(sTP_DEV.TP_Scan_Dir == R2L_D2U) {
                    printf("R2L_D2U\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[1][0] -
                                              XYpoint_Arr[0][0]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[2][1] -
                                              XYpoint_Arr[0][1]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[1][0] +
                                                      XYpoint_Arr[0][0])
                                    ) / 2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page -
                                     sTP_DEV.fYfac * (XYpoint_Arr[2][1] +
                                                      XYpoint_Arr[0][1])
                                    ) / 2;

                } else if(sTP_DEV.TP_Scan_Dir == L2R_U2D) {
                    printf("L2R_U2D\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][0] -
                                              XYpoint_Arr[1][0]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][1] -
                                              XYpoint_Arr[2][1]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[0][0] +
                                                      XYpoint_Arr[1][0])
                                    ) / 2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac *
                                     (XYpoint_Arr[0][1] + XYpoint_Arr[2][1])) / 2;
                } else if (sTP_DEV.TP_Scan_Dir == U2D_R2L) {
                    printf("U2D_R2L\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[1][1] - XYpoint_Arr[0][1]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[2][0] - XYpoint_Arr[0][0]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[1][1] +
                                                      XYpoint_Arr[0][1])
                                    ) / 2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page -
                                     sTP_DEV.fYfac * (XYpoint_Arr[2][0] +
                                                      XYpoint_Arr[0][0])
                                    ) / 2;
                } else {
                    printf("D2U_L2R\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][1] -
                                              XYpoint_Arr[1][1]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][0] -
                                              XYpoint_Arr[2][0]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[0][1] +
                                                      XYpoint_Arr[1][1])
                                    ) / 2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page -
                                     sTP_DEV.fYfac * (XYpoint_Arr[0][0] +
                                                      XYpoint_Arr[2][0])
                                    ) / 2;
                }
				
				printf("sTP_DEV.fXfac = %f \r\n", sTP_DEV.fXfac);
                printf("sTP_DEV.fYfac = %f \r\n", sTP_DEV.fYfac);
                printf("sTP_DEV.iXoff = %d \r\n", sTP_DEV.iXoff);
                printf("sTP_DEV.iYoff = %d \r\n", sTP_DEV.iYoff);
				
                //6.Calibration is successful
                LCD_Clear(LCD_BACKGROUND);
                GUI_DisString_EN(35, 110, "Touch Screen Adjust OK!",
                                 &Font16 , FONT_BACKGROUND , RED);
                Driver_Delay_ms(1000);
                LCD_Clear(LCD_BACKGROUND);
                return;
                //Exception handling,Reset  Initial value
            default	:
                cnt = 0;
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                             sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                TP_DrawCross(Mar_Val, Mar_Val, RED);
                GUI_DisString_EN(40, 26, "TP Need readjust!",
                                 &Font16 , FONT_BACKGROUND , RED);
                break;
            }
        }
    }
}



/*******************************************************************************
function:
		Use the default calibration factor
*******************************************************************************/
void TP_GetAdFac(void)
{
	if(LCD_2_8==id){ // 캘리브레이션 값
		sTP_DEV.fXfac = -0.090659;
        sTP_DEV.fYfac = 0.069858;
        sTP_DEV.iXoff = 355;
        sTP_DEV.iYoff = -20;
	}else{
		if(	sTP_DEV.TP_Scan_Dir == D2U_L2R ) { //SCAN_DIR_DFT = D2U_L2R
			sTP_DEV.fXfac = -0.132443 ;
			sTP_DEV.fYfac = 0.089997 ;
			sTP_DEV.iXoff = 516 ;
			sTP_DEV.iYoff = -22 ;
		} else if( sTP_DEV.TP_Scan_Dir == L2R_U2D ) {
			sTP_DEV.fXfac = 0.089697 ;
			sTP_DEV.fYfac = 0.134792 ;
			sTP_DEV.iXoff = -21 ;
			sTP_DEV.iYoff = -39 ;
		} else if( sTP_DEV.TP_Scan_Dir == R2L_D2U ) {
			sTP_DEV.fXfac = 0.089915 ;
			sTP_DEV.fYfac =  0.133178 ;
			sTP_DEV.iXoff = -22 ;
			sTP_DEV.iYoff = -38 ;
		} else if( sTP_DEV.TP_Scan_Dir == U2D_R2L ) {
			sTP_DEV.fXfac = -0.132906 ;
			sTP_DEV.fYfac = 0.087964 ;
			sTP_DEV.iXoff = 517 ;
			sTP_DEV.iYoff = -20 ;
		} else {
			LCD_Clear(LCD_BACKGROUND);
			GUI_DisString_EN(0, 60, "Does not support touch-screen \
							calibration in this direction",
							 &Font16, FONT_BACKGROUND, RED);
		}
	}
}

/*******************************************************************************
function:
		Paint the Delete key and paint color choose area
*******************************************************************************/
void TP_gesmain(void){ // 가로 X축 , 세로 Y축
    
    if(main_status){
    TP_Bmp_view(0,0,"b_main1.bmp");
    
    }else if(!main_status){
    TP_Bmp_view(0,0,"b_main2.bmp");
    }

    if(main_status){
        main_status=false;
    }else {
        main_status=true;
    }

    // TP_Bmp_view(0,0,"b_main2.bmp");
    
    printf("next\r\n");
}

// void TP_gesvallist(void)
// {

// }

// void TP_gessensor(void)
// {
//     //LCD_Clear(LCD_BACKGROUND);
//     GUI_DrawRectangle(53, 0, 267, 50, GBLUE, DRAW_FULL, DOT_PIXEL_1X1); // 상단바 BACK TIME LOGO
//     // GUI_DrawRectangle(0, 0, 50, 49, BLACK, 0, 2);
//     // GUI_DrawRectangle(270, 0, 320, 49, BLACK, 0, 2);
//     GUI_DrawLine(0,50,320,50,BLACK,0,1);
//     GUI_DisString_EN(5,15,"BACK",&Font16,GBLUE,BLACK);
//     // GUI_DisString_EN(275,15,"LOGO",&Font16,GBLUE,BLACK);
//     GUI_DisString_EN(70,15,"Sensor Check",&Font20,GBLUE,BLACK); //시간
//     GUI_DrawRectangle(0, 51, 320, 240, WHITE, DRAW_FULL, DOT_PIXEL_1X1);
    
//     GUI_DrawRectangle(20, 70, 139, 129, WHITE, 1, 1); // PWM START
//     GUI_DrawRectangle(20, 70, 140, 130, BLACK, 0, 1);
//     GUI_DisString_EN(40,90,"START",&Font24,WHITE,BLACK);

//     GUI_DrawRectangle(180 , 70, 299, 129, GBLUE, 1, 1); // PWM STOP
//     GUI_DrawRectangle(180 , 70, 300, 130, BLACK, 0, 1);
//     GUI_DisString_EN(205,90,"STOP",&Font24,GBLUE,BLACK);

//     GUI_DrawRectangle(20,165,299,219,GRAY,1,1); //20, 155, 140, 215, pwm 조절바
//     GUI_DrawRectangle(20,165,300,220,BLACK,0,1);

// }


// void TP_gessensor_pwm()
// {
//     if(pwmstatus){ // START
//         if(!sstart){ // 시작으로 바뀌었을때만 사용
            
//             GUI_DrawRectangle(20, 70, 139, 129, GBLUE, 1, 1); // PWM START
//             GUI_DisString_EN(40,90,"START",&Font24,GBLUE,BLACK);
//             GUI_DrawRectangle(180 , 70, 299, 129, WHITE, 1, 1); // PWM STOP
//             GUI_DisString_EN(205,90,"STOP",&Font24,WHITE,BLACK);
//             GUI_DrawRectangle(20,165,300-1,220-1,WHITE,1,1);

//         setPWM(1,1); 
//         setPWM(3,512); 
//         setPWM(5,1024); 
//         setPWM(7,2048);  
//         setPWM(9,3060);  
//         setPWM(11,4096); 
            
        
//         }
//         // GUI_DrawRectangle(20,165,pwmgui+21,220,GBLUE,1,1); //20, 155, 140, 215, pwm 조절바
//         // GUI_DisString_EN(70,185,pwmout,&Font24,WHITE,BLACK);
//     }
    
//     if(!pwmstatus){ // STOP
        
//         if(!sstop){
//             GUI_DrawRectangle(20, 70, 139, 129, WHITE, 1, 1); // PWM START
//             GUI_DisString_EN(40,90,"START",&Font24,WHITE,BLACK);
//             GUI_DrawRectangle(180 , 70, 299, 129, GBLUE, 1, 1); // PWM STOP
//             GUI_DisString_EN(205,90,"STOP",&Font24,GBLUE,BLACK);
//             GUI_DrawRectangle(20,165,300-1,220-1,GRAY,1,1);
            
//         setPWM(1,0);
//         setPWM(3,0);
//         setPWM(5,0);
//         setPWM(7,0);
//         setPWM(9,0);
//         setPWM(11,0);
//         }
//         // PWMOFF();
//     }
// }

// void TP_gessensor_pwm_bar(){


//     GUI_DrawRectangle(20,165,sTP_Draw.Xpoint,220-1,GBLUE,1,1); //프로그레스 바 업다운
//     GUI_DrawRectangle(sTP_Draw.Xpoint,165,299,220-1,WHITE,1,1);

// }

// void TP_gessetting(void)
// {
//     //LCD_Clear(LCD_BACKGROUND);
//     // TP_Bmp_view(0,0,2);
//     // TP_Bmp_view(0,0,"cat2.bmp");
//     GUI_DrawRectangle(53, 0, 267, 50, GBLUE, DRAW_FULL, DOT_PIXEL_1X1); // 상단바 BACK TIME LOGO
//     // GUI_DrawRectangle(0, 0, 50, 49, BLACK, 0, 2);
//     // GUI_DrawRectangle(270, 0, 320, 49, BLACK, 0, 2);
//     GUI_DrawLine(0,50,320,50,BLACK,0,1);
//     GUI_DisString_EN(3,15,"BACK",&Font16,GBLUE,BLACK);

//     GUI_DisString_EN(275,15,"LOGO",&Font16,GBLUE,BLACK);
//     GUI_DisString_EN(110,15,"SETTING",&Font20,GBLUE,BLACK); //시간
//     GUI_DrawRectangle(0, 51, 320, 240, WHITE, DRAW_FULL, DOT_PIXEL_1X1);

//     GUI_DrawRectangle(10, 60, 100, 140, GBLUE, 1, 1); // 시간변경
//     GUI_DrawRectangle(10, 60, 100, 140, BLACK, 0, 1);
//     GUI_DisString_EN(30,90,"START",&Font16,GBLUE,BLACK);

//     GUI_DrawRectangle(15+100, 60, 105+100, 140, GBLUE, 1, 1); // 단위 변경
//     GUI_DrawRectangle(15+100, 60, 105+100, 140, BLACK, 0, 1);
//     GUI_DisString_EN(128,90,"RESULT",&Font16,GBLUE,BLACK);

//     GUI_DrawRectangle(20+100+100, 60, 110+100+100, 140, GBLUE, 1, 1); // 메모리
//     GUI_DrawRectangle(20+100+100, 60, 110+100+100, 140, BLACK, 0, 1);
//     GUI_DisString_EN(233,90,"SENSOR",&Font16,GBLUE,BLACK);
    
//     GUI_DrawRectangle(10, 150, 100, 230, GBLUE, 1, 1); // 시간변경
//     GUI_DrawRectangle(10, 150, 100, 230, BLACK, 0, 1);
//     GUI_DisString_EN(30,185,"Blue",&Font16,GBLUE,BLACK);

//     GUI_DrawRectangle(15+100, 150, 105+100, 230, GBLUE, 1, 1); // 단위 변경
//     GUI_DrawRectangle(15+100, 150, 105+100, 230, BLACK, 0, 1);
//     GUI_DisString_EN(145,185,"Log",&Font16,GBLUE,BLACK);

//     GUI_DrawRectangle(20+100+100, 150, 110+100+100, 230, GBLUE, 1, 1); // 메모리
//     GUI_DrawRectangle(20+100+100, 150, 110+100+100, 230, BLACK, 0, 1);
//     GUI_DisString_EN(225,185,"HW info",&Font16,GBLUE,BLACK);
    

// }

// void TP_gesmpresultstart(void){
    
//         switch(start_status){
//             case 1:
//                 TP_Bmp_view(0,0,"start01.bmp");
//                 break;
//             case 2:
//                 TP_Bmp_view(0,0,"start01b.bmp");
//                 break;
//             case 3:
//                 TP_Bmp_view(0,0,"start02.bmp");
//                 break;
//             case 4:
//                 TP_Bmp_view(0,0,"start03.bmp");
                
//                 break;
//             case 5:
//                 start_status=0;
//                 pagestatus=0;
//                 TP_gesmain();
//                 break;
            
//         }
    
//     //LCD_Clear(LCD_BACKGROUND);
//     // GUI_DrawRectangle(53, 0, 267, 50, GBLUE, DRAW_FULL, DOT_PIXEL_1X1); // 상단바 BACK TIME LOGO
//     // // GUI_DrawRectangle(0, 0, 50, 49, BLACK, 0, 2);
//     // // GUI_DrawRectangle(270, 0, 320, 49, BLACK, 0, 2);
//     // GUI_DrawLine(0,50,320,50,BLACK,0,1);
//     // GUI_DisString_EN(3,15,"BACK",&Font16,GBLUE,BLACK);
//     // GUI_DisString_EN(120,15,"START",&Font20,GBLUE,BLACK); //시간
//     // GUI_DrawRectangle(0, 51, 320, 240, WHITE, DRAW_FULL, DOT_PIXEL_1X1);

//     // GUI_DisString_EN(25,90,"Please Set Frist mL",&Font20,WHITE,BLACK);
//     // GUI_DrawRectangle(103, 130, 103+17, 130+24, BLACK, 0, 1);
//     // GUI_DrawRectangle(103+(17*1), 130, 103+(17*2), 130+24, BLACK, 0, 1);
//     // GUI_DrawRectangle(103+(17*2), 130, 103+(17*3), 130+24, BLACK, 0, 1);
//     // GUI_DrawRectangle(103+(17*3), 130, 103+(17*4), 130+24, BLACK, 0, 1);
//     // GUI_DisString_EN(105,132,"0000 mL",&Font24,WHITE,BLACK);

//     // GUI_DrawRectangle(55, 175, 65+(17*12), 195+24, GBLUE, DRAW_FULL, 2);
//     // GUI_DisString_EN(70,185,"Micro START",&Font24,GBLUE,BLACK);
//     // GUI_DrawRectangle(55, 175, 65+(17*12), 195+24, BLACK, 0, 2);

// }

void TP_Bmp_view(uint8_t Xpoz, uint8_t Ypoz,const char* Bmpname){
    LCD_SetGramScanWay( 7 ); // BMP용 각도로 변경
	Storage_OpenReadFile(Xpoz,Ypoz,Bmpname);
	LCD_SetGramScanWay( 4 ); // 터치용 각도로 변경
    // printf("save_page1 \r\n");

}

void TP_Dialog(void)
{
	LCD_Clear(LCD_BACKGROUND);
	if(LCD_2_8==id){
			GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 60, 0,
							 "CLEAR", &Font16, RED, BLUE);
			GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 120, 0,
							 "AD", &Font16, RED, BLUE);
			GUI_DrawRectangle(0, 0, 15, 15, BLUE, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(20, 0, 35, 15, GREEN, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(40, 0, 55, 15, RED, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(60, 0, 75, 15, YELLOW, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(80, 0, 95, 15, BLACK, DRAW_FULL, DOT_PIXEL_1X1);
	}else{
		
		//Horizontal screen display
		if(sLCD_DIS.LCD_Dis_Column > sLCD_DIS.LCD_Dis_Page) {
			//Clear screen
			GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 60, 0,
							 "CLEAR", &Font16, RED, BLUE);
			//adjustment
			GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 120, 0,
							 "AD", &Font16, RED, BLUE);
			//choose the color
			GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 50, 20,
							  sLCD_DIS.LCD_Dis_Column, 70,
							  BLUE, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 50, 80,
							  sLCD_DIS.LCD_Dis_Column, 130,
							  GREEN, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 50, 140,
							  sLCD_DIS.LCD_Dis_Column, 190,
							  RED, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 50, 200,
							  sLCD_DIS.LCD_Dis_Column, 250,
							  YELLOW, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(sLCD_DIS.LCD_Dis_Column - 50, 260,
							  sLCD_DIS.LCD_Dis_Column, 310,
							  BLACK, DRAW_FULL, DOT_PIXEL_1X1);

		} else { //Vertical screen display
			GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 60, 0,
							 "CLEAR", &Font16, RED, BLUE);
			GUI_DisString_EN(sLCD_DIS.LCD_Dis_Column - 120, 0,
							 "AD", &Font24, RED, BLUE);
			GUI_DrawRectangle(20, 20, 70, 70, BLUE, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(80, 20, 130, 70, GREEN, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(140, 20, 190, 70, RED, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(200, 20, 250, 70, YELLOW, DRAW_FULL, DOT_PIXEL_1X1);
			GUI_DrawRectangle(260, 20, 310, 70, BLACK, DRAW_FULL, DOT_PIXEL_1X1);
		}
	}
}

/*******************************************************************************
function:
		Draw Board
*******************************************************************************/
void TP_DrawBoard(void)
{
	//sTP_DEV.chStatus &= ~(1 << 6);
    TP_Scan(0);

    if (sTP_DEV.chStatus & TP_PRESS_DOWN) {		//Press the button
        //Horizontal screen
        if (sTP_Draw.Xpoint < sLCD_DIS.LCD_Dis_Column &&
            //Dete/rmine whether the law is legal
            sTP_Draw.Ypoint < sLCD_DIS.LCD_Dis_Page) {

                if (((sTP_Draw.Xpoint > 0 && sTP_Draw.Xpoint < 50 ) &&  //이전
				    (sTP_Draw.Ypoint  > 0 && sTP_Draw.Ypoint < 50)) ){ 
                    TP_gesmain();      
                    // pagestatus=0;
                    // ox_sensing=false;
                    // start_status=0;
                    // if(sstart){
                    //     PWMOFF();
                    //     sstart=false;
                    //     sstop=true;
                    // 시료부 10,52 97 132 / 과산화수소 114,51 203,132 /nai 218,51 307 132 /시세 8,149 97,230 과세척 113,149 203,230 / n세척 218,150 307,230
                }else if(((sTP_Draw.Xpoint > 10 && sTP_Draw.Xpoint < 97 ) &&  //시료부
				        (sTP_Draw.Ypoint  > 52 && sTP_Draw.Ypoint < 132)) && (pagestatus==0)){ 
                        // TP_gesvallist();
                        TP_Bmp_view(0,0,"main.bmp");
                        pagestatus=1; //시작
                }else if(((sTP_Draw.Xpoint > 10 && sTP_Draw.Xpoint < 97 ) &&  //시료부 세척
				        (sTP_Draw.Ypoint  > 149 && sTP_Draw.Ypoint < 230)) && (pagestatus==0)){
                        // TP_gesmpresultstart();
                        // ox_sensing=true;         
                        // start_status=1;               
                        pagestatus=2;
                }else if(((sTP_Draw.Xpoint > 114 && sTP_Draw.Xpoint < 203 ) &&  //과산화수소
				        (sTP_Draw.Ypoint  > 51 && sTP_Draw.Ypoint < 132) )&& (pagestatus==0)){
                        // TP_gessensor();
                        // pwmgui=0;
                        pagestatus=3; //PWM 상태
                }else if(sTP_Draw.Xpoint > 113 && sTP_Draw.Xpoint < 203  &&  //과산화수소 세척
				        sTP_Draw.Ypoint  > 149 && sTP_Draw.Ypoint < 230 && pagestatus==0){
                        // TP_gessetting();
                        // getPCAmode(0x00);
                        // read_PCA_reg(0xfe,1);
                        pagestatus=4;
                }else if(((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 ) &&  // nai
				        (sTP_Draw.Ypoint  > 51 && sTP_Draw.Ypoint < 132)) && (pagestatus==0)){
                        // pwmstatus=1;
                        // // pwmgui=0;
                        // TP_gessensor_pwm(); 
                        // sstop=false;
                        // sstart=true;
                        pagestatus=5;
                }else if(((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 ) &&  // nai 세척
				        (sTP_Draw.Ypoint  > 150 && sTP_Draw.Ypoint < 230)) && (pagestatus==0)){
                        // pwmstatus=0;                        
                        // TP_gessensor_pwm();
                        // PWMOFF();
                        // pwmgui=0;                            
                        // sstop=true;
                        // sstart=false;
                        pagestatus=6;
                }else if(((sTP_Draw.Xpoint > 273 && sTP_Draw.Xpoint < 318 ) &&  // 다음 273,1 318,40 --------------------------------
				        (sTP_Draw.Ypoint  > 0 && sTP_Draw.Ypoint < 40))){
                        TP_gesmain();
                        
                }else if(((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 ) &&  // nai 세척
				        (sTP_Draw.Ypoint  > 150 && sTP_Draw.Ypoint < 230)) && (pagestatus==7)){
                        
                        pagestatus=6;
                }else if(((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 ) &&  // nai 세척
				        (sTP_Draw.Ypoint  > 150 && sTP_Draw.Ypoint < 230)) && (pagestatus==7)){
                        
                        pagestatus=6;
                }else if(((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 ) &&  // nai 세척
				        (sTP_Draw.Ypoint  > 150 && sTP_Draw.Ypoint < 230)) && (pagestatus==7)){
                        
                        pagestatus=6;
                }   

			//  if(LCD_2_8 == id){
				
			// 	if (sTP_Draw.Xpoint > (sLCD_DIS.LCD_Dis_Column - 60) &&
			// 			sTP_Draw.Ypoint < 16) {//Clear Board
			// 			TP_Dialog();
			// 	} else if(sTP_Draw.Xpoint > (sLCD_DIS.LCD_Dis_Column - 120) &&
			// 			  sTP_Draw.Xpoint < (sLCD_DIS.LCD_Dis_Column - 80) &&
			// 			  sTP_Draw.Ypoint < 16) { //afresh adjustment
			// 		TP_Adjust();
			// 		TP_Dialog();
			// 	} else if(sTP_Draw.Xpoint > 0 && sTP_Draw.Xpoint < 15 &&
			// 			  sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 15) {
			// 		sTP_Draw.Color = BLUE;
			// 	} else if(sTP_Draw.Xpoint > 20 && sTP_Draw.Xpoint < 35 &&
			// 			  sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 15) {
			// 		sTP_Draw.Color = GREEN;
			// 	} else if(sTP_Draw.Xpoint > 40 && sTP_Draw.Xpoint < 55 &&
			// 			  sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 15) {
			// 		sTP_Draw.Color = RED;
			// 	} else if(sTP_Draw.Xpoint > 60 && sTP_Draw.Xpoint < 75 &&
			// 			  sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 15) {
			// 		sTP_Draw.Color = YELLOW;
			// 	} else if(sTP_Draw.Xpoint > 80 && sTP_Draw.Xpoint < 95 &&
			// 			  sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 15) {
			// 		sTP_Draw.Color = BLACK;
			// 	} else {
			// 		GUI_DrawPoint(sTP_Draw.Xpoint, sTP_Draw.Ypoint,
			// 			  sTP_Draw.Color , DOT_PIXEL_1X1, DOT_FILL_RIGHTUP);
			// 	 }
			// }
        }
    }

    // if(sstart && (pagestatus==3))
    // {        
        
    //     // printf("OUTPWM: %d \r\n",pwmgui);
    //     // Driver_Delay_ms(500);
    // }else if(sstop){
    //     pwmgui=0;
        
    // }

}

/*******************************************************************************
function:
		Touch pad initialization
*******************************************************************************/
void TP_Init( LCD_SCAN_DIR Lcd_ScanDir )
{
    DEV_Digital_Write(TP_CS_PIN,1);

    sTP_DEV.TP_Scan_Dir = Lcd_ScanDir;

    TP_Read_ADC_XY(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint);
}


