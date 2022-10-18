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

static uint32_t pwmgui = 0;
extern LCD_DIS sLCD_DIS;
extern uint8_t id;
static TP_DEV sTP_DEV;
static TP_DRAW sTP_Draw;
static DEV_TIME settime;
uint16_t pagestatus = 7;
uint8_t sample_run = 0, h2o2_run = 0, nai_run = 0;
uint32_t clock_start1 = 0, clock_end1 = 0, diff_clock = 0;
bool sample_set = false, h2o2_set = false, nai_set = false;
char rtc_buf[256];
char *rtc_str = &rtc_buf[0];
datetime_t get_t = {
    .year = 2022,
    .month = 00,
    .day = 00,
    .dotw = 0, // 0 is Sunday, so 5 is Friday
    .hour = 00,
    .min = 00,
    .sec = 00};

bool main1 = true;
bool main2 = false;
bool main3 = false;

bool valve_view = false;
bool pwmstatus = false;
bool ox_sensing = false;
bool sstart = false;
bool sstop = false;
uint16_t s_count = 0;
uint8_t pwmval = 0;
uint32_t pwmout = 0;
uint8_t start_status = 0;
uint8_t i2c_writebuf[3];
/*m1=시료펌프 / m2=배수펌프 / m3=과산화수소펌프 / m4=NAI펌프*/
uint8_t m1_power = 12, m2_power = 12, m3_power = 12, m4_power = 12, v8m_power = 12;

int ycur = 35;

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

    // A cycle of at least 400ns.
    DEV_Digital_Write(TP_CS_PIN, 0);

    SPI4W_Write_Byte(CMD);
    Driver_Delay_us(200);

    //	dont write 0xff, it will block xpt2046
    // Data = SPI4W_Read_Byte(0Xff);
    Data = SPI4W_Read_Byte(0X00);
    Data <<= 8; // 7bit
    Data |= SPI4W_Read_Byte(0X00);
    // Data = SPI4W_Read_Byte(0Xff);
    Data >>= 3; // 5bit
    DEV_Digital_Write(TP_CS_PIN, 1);
    return Data;
}

/*******************************************************************************
function:
        Read the 5th channel value and exclude the maximum and minimum returns the average
parameter:
    Channel_Cmd :	0x90 :Read channel Y +
                    0xd0 :Read channel x +
*******************************************************************************/
#define READ_TIMES 5 // Number of readings
#define LOST_NUM 1   // Discard value
static uint16_t TP_Read_ADC_Average(uint8_t Channel_Cmd)
{
    uint8_t i, j;
    uint16_t Read_Buff[READ_TIMES];
    uint16_t Read_Sum = 0, Read_Temp = 0;
    // LCD SPI speed = 3 MHz
    spi_set_baudrate(SPI_PORT, 3000000);
    // Read and save multiple samples
    for (i = 0; i < READ_TIMES; i++)
    {
        Read_Buff[i] = TP_Read_ADC(Channel_Cmd);
        Driver_Delay_us(200);
    }
    // LCD SPI speed = 18 MHz
    spi_set_baudrate(SPI_PORT, 50 * 1000 * 1000);
    // Sort from small to large
    for (i = 0; i < READ_TIMES - 1; i++)
    {
        for (j = i + 1; j < READ_TIMES; j++)
        {
            if (Read_Buff[i] > Read_Buff[j])
            {
                Read_Temp = Read_Buff[i];
                Read_Buff[i] = Read_Buff[j];
                Read_Buff[j] = Read_Temp;
            }
        }
    }

    // Exclude the largest and the smallest
    for (i = LOST_NUM; i < READ_TIMES - LOST_NUM; i++)
        Read_Sum += Read_Buff[i];

    // Averaging
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
static void TP_Read_ADC_XY(uint16_t *pXCh_Adc, uint16_t *pYCh_Adc)
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
#define ERR_RANGE 50 // tolerance scope
static bool TP_Read_TwiceADC(uint16_t *pXCh_Adc, uint16_t *pYCh_Adc)
{
    uint16_t XCh_Adc1, YCh_Adc1, XCh_Adc2, YCh_Adc2;

    // Read the ADC values Read the ADC values twice
    TP_Read_ADC_XY(&XCh_Adc1, &YCh_Adc1);
    Driver_Delay_us(10);
    TP_Read_ADC_XY(&XCh_Adc2, &YCh_Adc2);
    Driver_Delay_us(10);

    // The ADC error used twice is greater than ERR_RANGE to take the average
    if (((XCh_Adc2 <= XCh_Adc1 && XCh_Adc1 < XCh_Adc2 + ERR_RANGE) ||
         (XCh_Adc1 <= XCh_Adc2 && XCh_Adc2 < XCh_Adc1 + ERR_RANGE)) &&
        ((YCh_Adc2 <= YCh_Adc1 && YCh_Adc1 < YCh_Adc2 + ERR_RANGE) ||
         (YCh_Adc1 <= YCh_Adc2 && YCh_Adc2 < YCh_Adc1 + ERR_RANGE)))
    {
        *pXCh_Adc = (XCh_Adc1 + XCh_Adc2) / 2;
        *pYCh_Adc = (YCh_Adc1 + YCh_Adc2) / 2;
        return true;
    }

    // The ADC error used twice is less than ERR_RANGE returns failed
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
    // In X, Y coordinate measurement, IRQ is disabled and output is low
    if (!DEV_Digital_Read(TP_IRQ_PIN))
    { // Press the button to press
        // Read the physical coordinates
        if (chCoordType)
        {
            TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint);
            // Read the screen coordinates
        }
        else if (TP_Read_TwiceADC(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint))
        {

            if (LCD_2_8 == id)
            { // 시작 좌표 설정
                sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Ypoint +
                                  sTP_DEV.iXoff;
                sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Xpoint +
                                  sTP_DEV.iYoff;
            }
            else
            {
                // DEBUG("(Xad,Yad) = %d,%d\r\n",sTP_DEV.Xpoint,sTP_DEV.Ypoint);
                if (sTP_DEV.TP_Scan_Dir == R2L_D2U)
                { // Converts the result to screen coordinates
                    sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Xpoint +
                                      sTP_DEV.iXoff;
                    sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Ypoint +
                                      sTP_DEV.iYoff;
                }
                else if (sTP_DEV.TP_Scan_Dir == L2R_U2D)
                {
                    sTP_Draw.Xpoint = sLCD_DIS.LCD_Dis_Column -
                                      sTP_DEV.fXfac * sTP_DEV.Xpoint -
                                      sTP_DEV.iXoff;
                    sTP_Draw.Ypoint = sLCD_DIS.LCD_Dis_Page -
                                      sTP_DEV.fYfac * sTP_DEV.Ypoint -
                                      sTP_DEV.iYoff;
                }
                else if (sTP_DEV.TP_Scan_Dir == U2D_R2L)
                {
                    sTP_Draw.Xpoint = sTP_DEV.fXfac * sTP_DEV.Ypoint +
                                      sTP_DEV.iXoff;
                    sTP_Draw.Ypoint = sTP_DEV.fYfac * sTP_DEV.Xpoint +
                                      sTP_DEV.iYoff;
                }
                else
                {
                    sTP_Draw.Xpoint = sLCD_DIS.LCD_Dis_Column -
                                      sTP_DEV.fXfac * sTP_DEV.Ypoint -
                                      sTP_DEV.iXoff;
                    sTP_Draw.Ypoint = sLCD_DIS.LCD_Dis_Page -
                                      sTP_DEV.fYfac * sTP_DEV.Xpoint -
                                      sTP_DEV.iYoff;
                }
                /// DEBUG("( x , y ) = %d,%d\r\n",sTP_Draw.Xpoint,sTP_Draw.Ypoint);
            }
        }
        if (0 == (sTP_DEV.chStatus & TP_PRESS_DOWN))
        { // Not being pressed
            sTP_DEV.chStatus = TP_PRESS_DOWN | TP_PRESSED;
            sTP_DEV.Xpoint0 = sTP_DEV.Xpoint;
            sTP_DEV.Ypoint0 = sTP_DEV.Ypoint;
        }
    }
    else
    {
        if (sTP_DEV.chStatus & TP_PRESS_DOWN)
        {                                  // 0x80
            sTP_DEV.chStatus &= ~(1 << 7); // 0x00
        }
        else
        {
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
    GUI_DrawLine(Xpoint - 12, Ypoint, Xpoint + 12, Ypoint,
                 Color, LINE_SOLID, DOT_PIXEL_1X1);
    GUI_DrawLine(Xpoint, Ypoint - 12, Xpoint, Ypoint + 12,
                 Color, LINE_SOLID, DOT_PIXEL_1X1);
    GUI_DrawPoint(Xpoint, Ypoint, Color, DOT_PIXEL_2X2, DOT_FILL_AROUND);
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
    if (LCD_2_8 == id)
    {
    }
    else
    {
        sFONT *TP_Font = &Font16;
        LENGTH TP_Dx = TP_Font->Width;

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

        GUI_DisNum(40 + 10 * TP_Dx, 240, hwFac, TP_Font, FONT_BACKGROUND, RED);
    }
}

/*******************************************************************************
function:
        Touch screen adjust
*******************************************************************************/
void TP_Adjust(void)
{
    uint8_t cnt = 0;
    uint16_t XYpoint_Arr[4][2];
    uint32_t Dx, Dy;
    uint16_t Sqrt1, Sqrt2;
    float Dsqrt;

    LCD_Clear(LCD_BACKGROUND);
    GUI_DisString_EN(0, 60, "Please use the stylus click the cross"
                            "on the screen. The cross will always move until"
                            "the screen adjustment is completed.",
                     &Font16, FONT_BACKGROUND, RED);

    uint8_t Mar_Val = 12;
    TP_DrawCross(Mar_Val, Mar_Val, RED);

    sTP_DEV.chStatus = 0;
    while (1)
    {
        TP_Scan(1);
        if ((sTP_DEV.chStatus & 0xC0) == TP_PRESSED)
        {
            sTP_DEV.chStatus &= ~(1 << 6);
            XYpoint_Arr[cnt][0] = sTP_DEV.Xpoint;
            XYpoint_Arr[cnt][1] = sTP_DEV.Ypoint;
            printf("X%d,Y%d = %d,%d\r\n", cnt, cnt, XYpoint_Arr[cnt][0], XYpoint_Arr[cnt][1]);
            cnt++;
            Driver_Delay_ms(200);

            switch (cnt)
            {
            case 1:
                // DEBUG("not touch TP_IRQ 2 = %d\r\n", GET_TP_IRQ);
                TP_DrawCross(Mar_Val, Mar_Val, WHITE);
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val, Mar_Val, RED);
                Driver_Delay_ms(200);
                break;
            case 2:
                // DEBUG("not touch TP_IRQ 3 = %d\r\n", GET_TP_IRQ);
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val, Mar_Val, WHITE);
                TP_DrawCross(Mar_Val, sLCD_DIS.LCD_Dis_Page - Mar_Val, RED);
                Driver_Delay_ms(200);
                break;
            case 3:
                // DEBUG("not touch TP_IRQ 4 = %d\r\n", GET_TP_IRQ);
                TP_DrawCross(Mar_Val, sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                             sLCD_DIS.LCD_Dis_Page - Mar_Val, RED);
                Driver_Delay_ms(200);
                break;
            case 4:

                // 1.Compare the X direction
                Dx = abs((int16_t)(XYpoint_Arr[0][0] -
                                   XYpoint_Arr[1][0])); // x1 - x2
                Dy = abs((int16_t)(XYpoint_Arr[0][1] -
                                   XYpoint_Arr[1][1])); // y1 - y2
                Dx *= Dx;
                Dy *= Dy;
                Sqrt1 = sqrt(Dx + Dy);

                Dx = abs((int16_t)(XYpoint_Arr[2][0] -
                                   XYpoint_Arr[3][0])); // x3 - x4
                Dy = abs((int16_t)(XYpoint_Arr[2][1] -
                                   XYpoint_Arr[3][1])); // y3 - y4
                Dx *= Dx;
                Dy *= Dy;
                Sqrt2 = sqrt(Dx + Dy);

                Dsqrt = (float)Sqrt1 / Sqrt2;
                if (Dsqrt < 0.95 || Dsqrt > 1.05 || Sqrt1 == 0 || Sqrt2 == 0)
                {
                    // DEBUG("Adjust X direction \r\n");
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
                                   XYpoint_Arr[2][0])); // x1 - x3
                Dy = abs((int16_t)(XYpoint_Arr[0][1] -
                                   XYpoint_Arr[2][1])); // y1 - y3
                Dx *= Dx;
                Dy *= Dy;
                Sqrt1 = sqrt(Dx + Dy);

                Dx = abs((int16_t)(XYpoint_Arr[1][0] -
                                   XYpoint_Arr[3][0])); // x2 - x4
                Dy = abs((int16_t)(XYpoint_Arr[1][1] -
                                   XYpoint_Arr[3][1])); // y2 - y4
                Dx *= Dx;
                Dy *= Dy;
                Sqrt2 = sqrt(Dx + Dy); //

                Dsqrt = (float)Sqrt1 / Sqrt2;
                if (Dsqrt < 0.95 || Dsqrt > 1.05)
                {
                    // DEBUG("Adjust Y direction \r\n");
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
                } //

                // 3.Compare diagonal
                Dx = abs((int16_t)(XYpoint_Arr[1][0] -
                                   XYpoint_Arr[2][0])); // x1 - x3
                Dy = abs((int16_t)(XYpoint_Arr[1][1] -
                                   XYpoint_Arr[2][1])); // y1 - y3
                Dx *= Dx;
                Dy *= Dy;
                Sqrt1 = sqrt(Dx + Dy); //;

                Dx = abs((int16_t)(XYpoint_Arr[0][0] -
                                   XYpoint_Arr[3][0])); // x2 - x4
                Dy = abs((int16_t)(XYpoint_Arr[0][1] -
                                   XYpoint_Arr[3][1])); // y2 - y4
                Dx *= Dx;
                Dy *= Dy;
                Sqrt2 = sqrt(Dx + Dy); //

                Dsqrt = (float)Sqrt1 / Sqrt2;
                if (Dsqrt < 0.95 || Dsqrt > 1.05)
                {
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

                // 4.Get the scale factor and offset
                // Get the scanning direction of the touch screen
                sTP_DEV.TP_Scan_Dir = sLCD_DIS.LCD_Scan_Dir;
                sTP_DEV.fXfac = 0;

                // According to the display direction to get
                // the corresponding scale factor and offset
                if (sTP_DEV.TP_Scan_Dir == R2L_D2U)
                {
                    printf("R2L_D2U\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[1][0] -
                                              XYpoint_Arr[0][0]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[2][1] -
                                              XYpoint_Arr[0][1]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[1][0] +
                                                      XYpoint_Arr[0][0])) /
                                    2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page -
                                     sTP_DEV.fYfac * (XYpoint_Arr[2][1] +
                                                      XYpoint_Arr[0][1])) /
                                    2;
                }
                else if (sTP_DEV.TP_Scan_Dir == L2R_U2D)
                {
                    printf("L2R_U2D\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][0] -
                                              XYpoint_Arr[1][0]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][1] -
                                              XYpoint_Arr[2][1]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[0][0] +
                                                      XYpoint_Arr[1][0])) /
                                    2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page - sTP_DEV.fYfac *
                                                                 (XYpoint_Arr[0][1] + XYpoint_Arr[2][1])) /
                                    2;
                }
                else if (sTP_DEV.TP_Scan_Dir == U2D_R2L)
                {
                    printf("U2D_R2L\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[1][1] - XYpoint_Arr[0][1]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[2][0] - XYpoint_Arr[0][0]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[1][1] +
                                                      XYpoint_Arr[0][1])) /
                                    2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page -
                                     sTP_DEV.fYfac * (XYpoint_Arr[2][0] +
                                                      XYpoint_Arr[0][0])) /
                                    2;
                }
                else
                {
                    printf("D2U_L2R\r\n");

                    sTP_DEV.fXfac = (float)(sLCD_DIS.LCD_Dis_Column - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][1] -
                                              XYpoint_Arr[1][1]);
                    sTP_DEV.fYfac = (float)(sLCD_DIS.LCD_Dis_Page - 2 * Mar_Val) /
                                    (int16_t)(XYpoint_Arr[0][0] -
                                              XYpoint_Arr[2][0]);

                    sTP_DEV.iXoff = (sLCD_DIS.LCD_Dis_Column -
                                     sTP_DEV.fXfac * (XYpoint_Arr[0][1] +
                                                      XYpoint_Arr[1][1])) /
                                    2;
                    sTP_DEV.iYoff = (sLCD_DIS.LCD_Dis_Page -
                                     sTP_DEV.fYfac * (XYpoint_Arr[0][0] +
                                                      XYpoint_Arr[2][0])) /
                                    2;
                }

                printf("sTP_DEV.fXfac = %f \r\n", sTP_DEV.fXfac);
                printf("sTP_DEV.fYfac = %f \r\n", sTP_DEV.fYfac);
                printf("sTP_DEV.iXoff = %d \r\n", sTP_DEV.iXoff);
                printf("sTP_DEV.iYoff = %d \r\n", sTP_DEV.iYoff);

                // 6.Calibration is successful
                LCD_Clear(LCD_BACKGROUND);
                GUI_DisString_EN(35, 110, "Touch Screen Adjust OK!",
                                 &Font16, FONT_BACKGROUND, RED);
                Driver_Delay_ms(1000);
                LCD_Clear(LCD_BACKGROUND);
                return;
                // Exception handling,Reset  Initial value
            default:
                cnt = 0;
                TP_DrawCross(sLCD_DIS.LCD_Dis_Column - Mar_Val,
                             sLCD_DIS.LCD_Dis_Page - Mar_Val, WHITE);
                TP_DrawCross(Mar_Val, Mar_Val, RED);
                GUI_DisString_EN(40, 26, "TP Need readjust!",
                                 &Font16, FONT_BACKGROUND, RED);
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
    if (LCD_2_8 == id)
    { // 캘리브레이션 값
        sTP_DEV.fXfac = -0.090659;
        sTP_DEV.fYfac = 0.069858;
        sTP_DEV.iXoff = 355;
        sTP_DEV.iYoff = -20;
    }
    else
    {
        if (sTP_DEV.TP_Scan_Dir == D2U_L2R)
        { // SCAN_DIR_DFT = D2U_L2R
            sTP_DEV.fXfac = -0.132443;
            sTP_DEV.fYfac = 0.089997;
            sTP_DEV.iXoff = 516;
            sTP_DEV.iYoff = -22;
        }
        else if (sTP_DEV.TP_Scan_Dir == L2R_U2D)
        {
            sTP_DEV.fXfac = 0.089697;
            sTP_DEV.fYfac = 0.134792;
            sTP_DEV.iXoff = -21;
            sTP_DEV.iYoff = -39;
        }
        else if (sTP_DEV.TP_Scan_Dir == R2L_D2U)
        {
            sTP_DEV.fXfac = 0.089915;
            sTP_DEV.fYfac = 0.133178;
            sTP_DEV.iXoff = -22;
            sTP_DEV.iYoff = -38;
        }
        else if (sTP_DEV.TP_Scan_Dir == U2D_R2L)
        {
            sTP_DEV.fXfac = -0.132906;
            sTP_DEV.fYfac = 0.087964;
            sTP_DEV.iXoff = 517;
            sTP_DEV.iYoff = -20;
        }
        else
        {
            LCD_Clear(LCD_BACKGROUND);
            GUI_DisString_EN(0, 60, "Does not support touch-screen \
							calibration in this direction",
                             &Font16, FONT_BACKGROUND, RED);
        }
    }
}

/*******************************************************************************
function: 메인 페이지 - 테스트,설정,밸브 이동
*******************************************************************************/
void TP_gesmain(void)
{ // 가로 X축 , 세로 Y축

    if (pagestatus >= 1 && pagestatus <= 6 || pagestatus == 7 || (pagestatus == 14 && !valve_view))
    {
        main1 = true;
        main2 = false;
        main3 = false;
    }
    else if (pagestatus >= 8 && pagestatus <= 13 || pagestatus == 0 || (pagestatus == 14 && !valve_view))
    {
        main1 = false;
        main2 = true;
        main3 = false;
    }

    if (valve_view)
    {
        main1 = false;
        main2 = false;
        main3 = true;
        valve_view = false;
    }

    if (main1)
    {
        TP_Bmp_view(0, 0, "b_main1.bmp");
        pagestatus = 0;
        return 0;
    }
    else if (main2)
    {
        TP_Bmp_view(0, 0, "b_main2.bmp");
        pagestatus = 7;
        return 0;
    }
    else if (main3)
    {
        TP_Bmp_view(0, 0, "b_main3.bmp");
        pagestatus = 14;
        return 0;
    }
    else
    {
    }
}

/*페이지 표시 및 기본 버튼 선택 표현*/
void TP_start_view(uint8_t pagenum)
{

    switch (pagenum)
    {
    case 1: // 전처리 테스트 인식 글자수 최대 14
        TP_Bmp_view(0, 0, "sp_s.bmp");

        TP_Bmp_num(250, 58, m1_power, false);
        GUI_DisString_EN(280, 65, "V", &Font20, WHITE, BLACK);
        TP_Bmp_num(250, 84, m2_power, false);
        GUI_DisString_EN(280, 90, "V", &Font20, WHITE, BLACK);
        GUI_DisString_EN(230, 117, "NULL", &Font20, WHITE, BLACK);

        GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1); // 정지 바탕색
        TP_Bmp_button(256, 177, 8);                         // 정지 표현
        break;
    case 2:
        TP_Bmp_view(0, 0, "sp_c.bmp");
        break;
    case 3:
        TP_Bmp_view(0, 0, "h2o2_s.bmp");

        TP_Bmp_num(250, 58, m3_power, false);
        GUI_DisString_EN(280, 65, "V", &Font20, WHITE, BLACK);
        TP_Bmp_num(250, 84, m2_power, false);
        GUI_DisString_EN(280, 90, "V", &Font20, WHITE, BLACK);
        GUI_DisString_EN(230, 117, "NULL", &Font20, WHITE, BLACK);

        GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1);
        TP_Bmp_button(256, 177, 8);
        break;
    case 4:
        TP_Bmp_view(0, 0, "h2o2_c.bmp");
        break;
    case 5:
        TP_Bmp_view(0, 0, "nai_s.bmp");

        TP_Bmp_num(250, 46, m1_power, false);
        GUI_DisString_EN(280, 52, "V", &Font20, WHITE, BLACK);
        TP_Bmp_num(250, 70, m1_power, false);
        GUI_DisString_EN(280, 76, "V", &Font20, WHITE, BLACK);
        TP_Bmp_num(250, 96, m2_power, false);
        GUI_DisString_EN(280, 102, "V", &Font20, WHITE, BLACK);
        GUI_DisString_EN(230, 126, "NULL", &Font20, WHITE, BLACK);

        GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1);
        TP_Bmp_button(256, 177, 8);
        break;
    case 6:
        TP_Bmp_view(0, 0, "nai_c.bmp");
        break;
    case 8: // 전처리 설정
        TP_Bmp_view(0, 0, "sp_s_p.bmp");
        break;
    case 9:
        TP_Bmp_view(0, 0, "sp_c_p.bmp");
        break;
    case 10:
        TP_Bmp_view(0, 0, "h2o2_s_p.bmp");
        break;
    case 11:
        TP_Bmp_view(0, 0, "h2o2_c_p.bmp");
        break;
    case 12:
        TP_Bmp_view(0, 0, "nai_s_p.bmp");
        break;
    case 13:
        TP_Bmp_view(0, 0, "nai_c_p.bmp");
        break;
    case 14: // 밸브
        TP_Bmp_view(0, 0, "nai_c_p.bmp");
        break;

    default:

        break;
    }
}
/*
Function: 선택된 페이지 BMP 파일 보기
*/
void TP_Bmp_view(uint16_t Xpoz, uint16_t Ypoz, const char *Bmpname) // 배경
{
    LCD_SetGramScanWay(7); // BMP용 각도로 변경
    Storage_OpenReadFile(Xpoz, Ypoz, Bmpname);
    LCD_SetGramScanWay(4); // 터치용 각도로 변경
    // printf("save_page1 \r\n");
}
/*
Function: 버튼 Bmp 표현
*/
void TP_Bmp_button(uint16_t Xpoz, uint16_t Ypoz, uint8_t btn_num) // 버튼
{
    LCD_SetGramScanWay(7);
    show_button(Xpoz, 240 - Ypoz - 20, btn_num);
    LCD_SetGramScanWay(4);
    // printf("save_page1 \r\n");
}

/*
Function: 숫자 Bmp 표현
*/
void TP_Bmp_num(uint16_t Xpoz, uint16_t Ypoz, uint8_t Num, bool back_g) // 숫자
{
    LCD_SetGramScanWay(7);
    show_num(Xpoz, 240 - Ypoz - 20, Num, back_g);
    LCD_SetGramScanWay(4);
}
/*******************************************************************************
function:
        Draw Board
*******************************************************************************/
void TP_DrawBoard(void)
{
    // sTP_DEV.chStatus &= ~(1 << 6);
    TP_Scan(0);

    if (sTP_DEV.chStatus & TP_PRESS_DOWN)
    { // Press the button
        // Horizontal screen
        if (sTP_Draw.Xpoint < sLCD_DIS.LCD_Dis_Column &&
            // Dete/rmine whether the law is legal
            sTP_Draw.Ypoint < sLCD_DIS.LCD_Dis_Page)
        {
            // printf("\n x:%d,y:%d \r\n", sTP_Draw.Xpoint, sTP_Draw.Ypoint);

            // 특정 위치 선택시 기능동작
            if ((sTP_Draw.Xpoint > 0 && sTP_Draw.Xpoint < 50 && //이전
                 sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 50) &&
                pagestatus <= 14)
            {
                TP_gesmain();

                // 시료부 10,52 97 132 / 과산화수소 114,51 203,132 /nai 218,51 307 132 /시세 8,149 97,230 과세척 113,149 203,230 / n세척 218,150 307,230
            }
            else if ((sTP_Draw.Xpoint > 10 && sTP_Draw.Xpoint < 97 && //시료부
                      sTP_Draw.Ypoint > 52 && sTP_Draw.Ypoint < 132) &&
                     pagestatus == 0)
            {
                pagestatus = 1;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 10 && sTP_Draw.Xpoint < 97 && //시료부 세척
                      sTP_Draw.Ypoint > 149 && sTP_Draw.Ypoint < 230) &&
                     (pagestatus == 0))
            {
                pagestatus = 2;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 114 && sTP_Draw.Xpoint < 203 && //과산화수소
                      sTP_Draw.Ypoint > 51 && sTP_Draw.Ypoint < 132) &&
                     (pagestatus == 0))
            {
                pagestatus = 3;
                TP_start_view(pagestatus);
            }
            else if (sTP_Draw.Xpoint > 113 && sTP_Draw.Xpoint < 203 && //과산화수소 세척
                     sTP_Draw.Ypoint > 149 && sTP_Draw.Ypoint < 230 && pagestatus == 0)
            {
                // TP_gessetting();
                // getPCAmode(0x00);
                // read_PCA_reg(0xfe,1);
                pagestatus = 4;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 && // nai
                      sTP_Draw.Ypoint > 51 && sTP_Draw.Ypoint < 132) &&
                     pagestatus == 0)
            {
                pagestatus = 5;
                TP_start_view(pagestatus);
            }
            else if (((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307) && // nai 세척
                      (sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 230)) &&
                     pagestatus == 0)
            {
                pagestatus = 6;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 273 && sTP_Draw.Xpoint < 320 && // 밸브 273,1 318,40  --------------------------------
                      sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 40) &&
                     (pagestatus == 0 || pagestatus == 7))
            {
                valve_view = true;
                printf("valve\r\n");
                TP_gesmain();
            }
            else if ((sTP_Draw.Xpoint > 10 && sTP_Draw.Xpoint < 97 && //시료부 설정
                      sTP_Draw.Ypoint > 52 && sTP_Draw.Ypoint < 132) &&
                     pagestatus == 7)
            {
                pagestatus = 8;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 10 && sTP_Draw.Xpoint < 97 && //시료부 세척
                      sTP_Draw.Ypoint > 149 && sTP_Draw.Ypoint < 230) &&
                     (pagestatus == 7))
            {
                pagestatus = 9;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 114 && sTP_Draw.Xpoint < 203 && //과산화수소
                      sTP_Draw.Ypoint > 51 && sTP_Draw.Ypoint < 132) &&
                     (pagestatus == 7))
            {
                pagestatus = 10;
                TP_start_view(pagestatus);
            }
            else if (sTP_Draw.Xpoint > 113 && sTP_Draw.Xpoint < 203 && //과산화수소 세척
                     sTP_Draw.Ypoint > 149 && sTP_Draw.Ypoint < 230 && pagestatus == 7)
            {
                pagestatus = 11;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307 && // nai
                      sTP_Draw.Ypoint > 51 && sTP_Draw.Ypoint < 132) &&
                     pagestatus == 7)
            {
                pagestatus = 12;
                TP_start_view(pagestatus);
            }
            else if (((sTP_Draw.Xpoint > 218 && sTP_Draw.Xpoint < 307) && // nai 세척
                      (sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 230)) &&
                     pagestatus == 7)
            {
                pagestatus = 13;
                TP_start_view(pagestatus);
            }
            else if ((sTP_Draw.Xpoint > 273 && sTP_Draw.Xpoint < 320 && // 밸브 273,1 318,40 --------------------------------
                      sTP_Draw.Ypoint > 0 && sTP_Draw.Ypoint < 40) &&
                     pagestatus == 14)
            {
                pagestatus = 13;
                TP_gesmain();
            }
            // printf("pagestatus : %d\r\n", pagestatus);

            Run_page(pagestatus); // 페이지에 따른 버튼 활성화

            // 오동작으로 다른페이지로 넘어가지 않도록 안쓰는 좌표값으로 변경
            sTP_Draw.Xpoint = 160;
            sTP_Draw.Ypoint = 1;
        }
    }
    /* 전처리 동작 */
    Run_page_func(pagestatus);
    // if(sstart && (pagestatus==3))
    // {

    //     // printf("OUTPWM: %d \r\n",pwmgui);
    //     // Driver_Delay_ms(500);
    // }else if(sstop){
    //     pwmgui=0;

    // }
}

void Run_page_func(uint8_t page_num)
{
    switch (page_num)
    {
    case 1: // 시료부
        /*
        사용 모터 M1,M2,V8M
        사용 밸브 V1,V2,V3,V5
        */
        switch (sample_run)
        {
        case STOP: // stop
            if (sample_set)
            {
                sample_set = false;
                setMotor(V8M, MOTOR_OFF);
                setMotor(M1, MOTOR_OFF);
                setMotor(M2, MOTOR_OFF);
                setValve(V1, VALVE_OFF);
                setValve(V2, VALVE_OFF);
                setValve(V3, VALVE_OFF);
                setValve(V5, VALVE_OFF);

                GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 8);                         // 정지 표현
                GUI_DrawRectangle(20, 155, 77, 220, WHITE, 1, 1);   // 투입 바탕색
                TP_Bmp_button(30, 177, 1);                          // 투입 글씨
                GUI_DrawRectangle(97, 155, 154, 220, WHITE, 1, 1);  // 투입 바탕색
                TP_Bmp_button(107, 177, 2);                         // 투입 글씨
                GUI_DrawRectangle(172, 155, 229, 220, WHITE, 1, 1); // 투입 바탕색
                TP_Bmp_button(182, 177, 3);                         // 투입 글씨
            }
            break;

        case INJECT: // inject 투입
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            TP_Bmp_num(50, 156, diff_clock, true);

            if (!sample_set)
            {
                sample_set = true;
                setMotor(M1, m1_power);
                setMotor(M2, m2_power);
                setValve(V1, VALVE_ON);
                setValve(V2, VALVE_ON);
                setValve(V3, VALVE_OFF);
            }
            else if (diff_clock > 170 && diff_clock < 180) //투입 종료 잔여액 배출
            {
                setMotor(M1, MOTOR_OFF);
                setValve(V1, VALVE_OFF);
            }
            else if (diff_clock >= 180) // 3분후에 정지
            {
                sample_run = STOP;
            }

            break;

        case DRAIN: // drain 배수
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            TP_Bmp_num(130, 156, diff_clock, true);

            if (!sample_set)
            {
                sample_set = true;
                setMotor(M2, m2_power);
                setValve(V2, VALVE_ON);
                setValve(V3, VALVE_OFF);
            }
            else if (diff_clock >= 10)
            {
                sample_run = STOP;
            }
            break;

        case CLEAN: // clean 세척
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            TP_Bmp_num(205, 156, diff_clock, true);

            if (!sample_set)
            {
                sample_set = true;
                setMotor(V8M, 12); // 노즐 모터 동작
                setMotor(M1, 12);
                setMotor(M2, 12);
                setValve(V1, VALVE_ON);
                setValve(V2, VALVE_ON);
                setValve(V3, VALVE_OFF);
                setValve(V5, VALVE_OFF); // 시료부 세척
            }
            else if (diff_clock > 5 && diff_clock <= 10)
            {
                setValve(V5, VALVE_ON); // 반응조 세척
            }
            else if (diff_clock > 10 && diff_clock <= 15)
            {
                setMotor(V8M, MOTOR_OFF);
                setValve(V5, VALVE_OFF);
            }
            else if (diff_clock > 15)
            {
                sample_run = STOP;
            }
            break;

        default:
            break;
        }
        break;
    case 3: // 과산화수소
        switch (h2o2_run)
        {
        case 0: // stop
            break;
        case 1: // inject
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;
        case 2: // drain
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;
        case 3: // clean
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;

        default:
            break;
        }
        break;
    case 5: // NAI
        switch (nai_run)
        {
        case 0: // stop
            break;
        case 1: // inject
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;
        case 2: // drain
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;
        case 3: // clean
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;
        case 4: // overflow
            rtc_get_datetime(&get_t);
            clock_end1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; //진행시간 확인
            diff_clock = clock_end1 - clock_start1;
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}
/*
부팅후 동작시간 실시간 확인가능
4300초 까지밖에 측정이 안되어 장시간 사용불가
*/
clock_t clock_mp()
{
    return (clock_t)time_us_64() / 10000;
    // clock_start1 = clock_mp() / CLOCKS_PER_SEC;
}

/*
각 페이지별 버튼 기능 및 동작
16 150 / 80 223 투입 - 93 150 / 157 223 배수 - 170 150 / 230 223 세척 - 240 150/ 305 223 정지
*/
void Run_page(uint8_t page_num)
{
    if (page_num == 1) // 시료부
    {
        if ((sTP_Draw.Xpoint > 16 && sTP_Draw.Xpoint < 80 && // 시료부 투입
             sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (sample_run == 0)
            {
                sample_run = 1;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);

                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(20, 155, 77, 220, BLACK, 1, 1); // 투입 바탕색
                TP_Bmp_button(30, 177, 5);                        // 투입 글씨
            }

            printf("\r%d %d %d \n\r %d  \r\n", get_t.hour, get_t.min, get_t.sec, (get_t.hour * 3600) + (get_t.min * 60) + get_t.sec);
            // datetime_to_str(rtc_str, sizeof(rtc_buf), &get_t); // 시간값 문자열로 변환
            // printf("\r%s      \r\n", rtc_str);
        }
        else if ((sTP_Draw.Xpoint > 93 && sTP_Draw.Xpoint < 157 && // 시료부 배수
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (sample_run == 0)
            {
                sample_run = 2;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(97, 155, 154, 220, BLACK, 1, 1); // 배수 바탕색
                TP_Bmp_button(107, 177, 6);                        // 배수 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 170 && sTP_Draw.Xpoint < 230 && // 시료부 세척
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (sample_run == 0)
            {
                sample_run = 3;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(172, 155, 229, 220, BLACK, 1, 1); // 세척 바탕색
                TP_Bmp_button(182, 177, 7);                         // 세척 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 240 && sTP_Draw.Xpoint < 305 && // 정지
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (sample_run != 0)
            {
                sample_run = 0;
                GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 8);                         // 정지 표현

                GUI_DrawRectangle(20, 155, 77, 220, WHITE, 1, 1);   // 투입 바탕색
                TP_Bmp_button(30, 177, 1);                          // 투입 글씨
                GUI_DrawRectangle(97, 155, 154, 220, WHITE, 1, 1);  // 투입 바탕색
                TP_Bmp_button(107, 177, 2);                         // 투입 글씨
                GUI_DrawRectangle(172, 155, 229, 220, WHITE, 1, 1); // 투입 바탕색
                TP_Bmp_button(182, 177, 3);                         // 투입 글씨
            }
        }
    }
    else if (page_num == 3) // 과산화수소
    {
        if ((sTP_Draw.Xpoint > 16 && sTP_Draw.Xpoint < 80 && // 시료부 투입
             sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (h2o2_run == 0)
            {
                h2o2_run = 1;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(20, 155, 77, 220, BLACK, 1, 1); // 투입 바탕색
                TP_Bmp_button(30, 177, 5);                        // 투입 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 93 && sTP_Draw.Xpoint < 157 && // 시료부 배수
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (h2o2_run == 0)
            {
                h2o2_run = 2;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(97, 155, 154, 220, BLACK, 1, 1); // 배수 바탕색
                TP_Bmp_button(107, 177, 6);                        // 배수 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 170 && sTP_Draw.Xpoint < 230 && // 시료부 세척
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (h2o2_run == 0)
            {
                h2o2_run = 3;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(172, 155, 229, 220, BLACK, 1, 1); // 세척 바탕색
                TP_Bmp_button(182, 177, 7);                         // 세척 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 240 && sTP_Draw.Xpoint < 305 && // 정지
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (h2o2_run != 0)
            {
                h2o2_run = 0;
                GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 8);                         // 정지 표현

                GUI_DrawRectangle(20, 155, 77, 220, WHITE, 1, 1);   // 투입 바탕색
                TP_Bmp_button(30, 177, 1);                          // 투입 글씨
                GUI_DrawRectangle(97, 155, 154, 220, WHITE, 1, 1);  // 투입 바탕색
                TP_Bmp_button(107, 177, 2);                         // 투입 글씨
                GUI_DrawRectangle(172, 155, 229, 220, WHITE, 1, 1); // 투입 바탕색
                TP_Bmp_button(182, 177, 3);                         // 투입 글씨
            }
        }
    }
    else if (page_num == 5) // NAI
    {
        if ((sTP_Draw.Xpoint > 16 && sTP_Draw.Xpoint < 80 && // 시료부 투입
             sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (nai_run == 0)
            {
                nai_run = 1;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(20, 155, 77, 220, BLACK, 1, 1); // 투입 바탕색
                TP_Bmp_button(30, 177, 5);                        // 투입 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 93 && sTP_Draw.Xpoint < 157 && // 시료부 배수
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (nai_run == 0)
            {
                nai_run = 2;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(97, 155, 154, 220, BLACK, 1, 1); // 배수 바탕색
                TP_Bmp_button(107, 177, 6);                        // 배수 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 170 && sTP_Draw.Xpoint < 230 && // 시료부 세척
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (nai_run == 0)
            {
                nai_run = 3;
                rtc_get_datetime(&get_t);                                      // RTC 값 get
                clock_start1 = get_t.hour * 3600 + get_t.min * 60 + get_t.sec; // 시작 초로 저장
                printf("start clock : %d\r\n", clock_start1);
                GUI_DrawRectangle(245, 155, 302, 220, WHITE, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 4);                         // 정지 표현

                GUI_DrawRectangle(172, 155, 229, 220, BLACK, 1, 1); // 세척 바탕색
                TP_Bmp_button(182, 177, 7);                         // 세척 글씨
            }
        }
        else if ((sTP_Draw.Xpoint > 240 && sTP_Draw.Xpoint < 305 && // 정지
                  sTP_Draw.Ypoint > 150 && sTP_Draw.Ypoint < 223))
        {
            if (nai_run != 0)
            {
                nai_run = 0;

                GUI_DrawRectangle(245, 155, 302, 220, BLACK, 1, 1); // 정지 바탕색
                TP_Bmp_button(256, 177, 8);                         // 정지 표현

                GUI_DrawRectangle(20, 155, 77, 220, WHITE, 1, 1);   // 투입 바탕색
                TP_Bmp_button(30, 177, 1);                          // 투입 글씨
                GUI_DrawRectangle(97, 155, 154, 220, WHITE, 1, 1);  // 투입 바탕색
                TP_Bmp_button(107, 177, 2);                         // 투입 글씨
                GUI_DrawRectangle(172, 155, 229, 220, WHITE, 1, 1); // 투입 바탕색
                TP_Bmp_button(182, 177, 3);                         // 투입 글씨
            }
        }
    }
}

/*******************************************************************************
function:
        Touch pad initialization
*******************************************************************************/
void TP_Init(LCD_SCAN_DIR Lcd_ScanDir)
{
    DEV_Digital_Write(TP_CS_PIN, 1);

    sTP_DEV.TP_Scan_Dir = Lcd_ScanDir;

    TP_Read_ADC_XY(&sTP_DEV.Xpoint, &sTP_DEV.Ypoint);
}
