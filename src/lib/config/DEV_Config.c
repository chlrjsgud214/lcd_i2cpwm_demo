/*****************************************************************************
* | File      	:	DEV_Config.c
* | Author      :   Waveshare team
* | Function    :	Show SDcard BMP picto LCD 
* | Info        :
*   Provide the hardware underlying interface	 
*----------------
* |	This version:   V1.0
* | Date        :   2018-01-11
* | Info        :   Basic version
*
******************************************************************************/
#include "DEV_Config.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"


static uint32_t slice_num;
static uint32_t slice_0;
static uint32_t slice_1;
static uint32_t slice_2;

void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
	gpio_put(Pin, Value);
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
	return gpio_get(Pin);
}

/**
 * GPIO Mode
**/
void DEV_GPIO_Mode(UWORD Pin, UWORD Mode)
{
    gpio_init(Pin);
    if(Mode == 0 || Mode == GPIO_IN) {
        gpio_set_dir(Pin, GPIO_IN);
    } else {
        gpio_set_dir(Pin, GPIO_OUT);
    }
}

void DEV_GPIO_Init(void)
{
	DEV_GPIO_Mode(LCD_RST_PIN,GPIO_OUT);
    DEV_GPIO_Mode(LCD_DC_PIN, GPIO_OUT);
    DEV_GPIO_Mode(LCD_BKL_PIN, GPIO_OUT);
    DEV_GPIO_Mode(LCD_CS_PIN, GPIO_OUT);
    DEV_GPIO_Mode(TP_CS_PIN,GPIO_OUT);
    DEV_GPIO_Mode(TP_IRQ_PIN,GPIO_IN);
    DEV_GPIO_Mode(SD_CS_PIN,GPIO_OUT);
	  gpio_set_pulls(TP_IRQ_PIN,true,false);

    DEV_Digital_Write(TP_CS_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 1);
    DEV_Digital_Write(LCD_BKL_PIN, 1);
    DEV_Digital_Write(SD_CS_PIN, 1);
}

void SENSOR_GPIO_Init(void)
{
  DEV_GPIO_Mode(SENSOR_IN0,GPIO_IN);
  DEV_GPIO_Mode(SENSOR_IN1,GPIO_IN);
  DEV_GPIO_Mode(SENSOR_IN2,GPIO_IN);
  DEV_GPIO_Mode(SENSOR_IN3,GPIO_IN);
  DEV_GPIO_Mode(SENSOR_IN4,GPIO_IN);

  gpio_set_pulls(SENSOR_IN0,true,false);
  gpio_set_pulls(SENSOR_IN1,true,false);
  gpio_set_pulls(SENSOR_IN2,true,false);
  gpio_set_pulls(SENSOR_IN3,true,false);
  gpio_set_pulls(SENSOR_IN4,true,false);

  return 0;
}

/********************************************************************************
function:	PWM Init
note:
	PWM 함수
********************************************************************************/
void DEV_PWM_Init(void)
{	
	gpio_set_function(PWM_LED, GPIO_FUNC_PWM);
	gpio_set_function(PWM_CH0, GPIO_FUNC_PWM);
	gpio_set_function(PWM_CH1, GPIO_FUNC_PWM);
  slice_num = pwm_gpio_to_slice_num(PWM_LED); // 2B
  slice_0 = pwm_gpio_to_slice_num(PWM_CH0); // A0
  slice_1 = pwm_gpio_to_slice_num(PWM_CH1); // B0

  pwm_set_clkdiv(slice_num, clock_get_hz(clk_sys)/1000000); //133,000,000
  pwm_set_clkdiv(slice_0, clock_get_hz(clk_sys)/1000000); //133,000,000
  pwm_set_clkdiv(slice_1, clock_get_hz(clk_sys)/1000000); //133,000,000

  pwm_set_wrap(slice_num, 1000);
  pwm_set_wrap(slice_0, 1000);
  pwm_set_wrap(slice_1, 1000);
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_0, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_1, PWM_CHAN_B, 0);
  pwm_set_enabled(slice_num, true);
  pwm_set_enabled(slice_0, true);
  pwm_set_enabled(slice_1, true);
}
 

void PWMON(uint16_t val)
{
  pwm_set_wrap(slice_num, 255);
  pwm_set_wrap(slice_0, 255);
  pwm_set_wrap(slice_1, 255);
  
//   pwm_duty = cmap(val, 0, 255, 0, 1000);

  pwm_set_chan_level(slice_num, PWM_CHAN_B, val);
  pwm_set_chan_level(slice_0, PWM_CHAN_A, val);
  pwm_set_chan_level(slice_1, PWM_CHAN_B, val);

}

void PWMOFF(void)
{  
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
  pwm_set_chan_level(slice_0, PWM_CHAN_A, 0);
  pwm_set_chan_level(slice_1, PWM_CHAN_B, 0);
}
/********************************************************************************
function:	System Init
note:
	Initialize the communication method
********************************************************************************/
uint8_t System_Init(void)
{
	set_sys_clock_khz(250000, true); // 250Mhz
	stdio_init_all();
  // rtc_init();

  sleep_us(64);
	DEV_GPIO_Init();
	spi_init(SPI_PORT,4*1000*1000);
	gpio_set_function(LCD_CLK_PIN,GPIO_FUNC_SPI);
	gpio_set_function(LCD_MOSI_PIN,GPIO_FUNC_SPI);
	gpio_set_function(LCD_MISO_PIN,GPIO_FUNC_SPI);

    return 0;
}

void System_Exit(void)
{

}

/*********************************************
function:	Hardware interface
note:
	SPI4W_Write_Byte(value) : 
		Register hardware SPI
*********************************************/	
uint8_t SPI4W_Write_Byte(uint8_t value)                                    
{   
	uint8_t rxDat;
	spi_write_read_blocking(spi1,&value,&rxDat,1);
    return rxDat;
}

uint8_t SPI4W_Read_Byte(uint8_t value)                                    
{
	return SPI4W_Write_Byte(value);
}

/********************************************************************************
function:	Delay function
note:
	Driver_Delay_ms(xms) : Delay x ms
	Driver_Delay_us(xus) : Delay x us
********************************************************************************/
void Driver_Delay_ms(uint32_t xms)
{
	sleep_ms(xms);
}

void Driver_Delay_us(uint32_t xus)
{
	int j;
    for(j=xus; j > 0; j--);
}
