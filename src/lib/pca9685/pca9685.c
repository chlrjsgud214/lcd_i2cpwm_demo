

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pca9685.h"
#include "DEV_Config.h"

// Setup registers
const uint8_t PCA9685_MODE1 = 0x00;
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA
#define PWMALL_OFF_L 0xFC
#define PWMALL_OFF_H 0xFD
// #define PCA_ADDR 0x40
#define PIN_ALL 16
static uint8_t pca_addr =0x40;


// // Declare
// static void myPwmWrite(struct wiringPiNodeStruct *node, int pin, int value);
// static void myOnOffWrite(struct wiringPiNodeStruct *node, int pin, int value);
// static int myOffRead(struct wiringPiNodeStruct *node, int pin);
// static int myOnRead(struct wiringPiNodeStruct *node, int pin);
// int baseReg(int pin);



/**
 * Setup a PCA9685 device with wiringPi.
 *  
 * pinBase: 	Use a pinBase > 64, eg. 300
 * i2cAddress:	The default address is 0x40
 * freq:		Frequency will be capped to range [40..1000] Hertz. Try 50 for servos
 */
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int pca_i2c_init()
{
	uint8_t setBuf[2]={0x00,0x00};
	i2c_init(PICO_i2c1, 100 * 1000);
    gpio_set_function(PICO_I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C1_SDA_PIN);
    gpio_pull_up(PICO_I2C1_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_I2C1_SDA_PIN, PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C));
	
	getPCAmode(0x00); // 기본상태로 wake 명령 ,serial로 addr scan
	// i2c_write_blocking(PICO_i2c1,pca_addr,)
	// setPWMFreq(4096);
	// i2c_bus_scan();
}

void i2c_bus_scan()
{	
	printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(PICO_i2c1, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    // return 0;

}

void setPWMFreq(int freq)
{
// 	uint8_t rxdata;
// 	uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
// 	uint8_t buf[2];
// 	// int settings = i2c_read_blocking(PICO_i2c1,PCA_ADDR, PCA9685_MODE1) & 0x7F;	// Set restart bit to 0
// 	int sleep	= settings | 0x10;									// Set sleep bit to 1
// 	int wake 	= settings & 0xEF;									// Set sleep bit to 0
// 	int restart = wake | 0x80;										// Set restart bit to 1

// // 	// Go to sleep, set prescale and wake up again.
// 	buf[0]=PCA9685_MODE1;
// 	buf[1]=sleep;
// 	i2c_write_blocking(PICO_i2c1,PCA_ADDR,buf,2,true);
// 	buf[0]=PRE_SCALE;
// 	buf[1]=prescale_val;
// 	i2c_write_blocking(PICO_i2c1,PCA_ADDR,buf,2,true);
// 	buf[0]=PCA9685_MODE1;
// 	buf[1]=wake;
// 	i2c_write_blocking(PICO_i2c1,PCA_ADDR,buf,2,true);
// 	// i2c_write_blocking(fd, PCA9685_PRESCALE, prescale);
// 	// i2c_write_blocking(fd, PCA9685_MODE1, wake);
// 	printf("Freq ok\n");

// // 	// Now wait a millisecond until oscillator finished stabilizing and restart PWM.
// 	delay(1);
// 	buf[0]=PCA9685_MODE1;
// 	buf[1]=restart;
// 	i2c_write_blocking(PICO_i2c1,PCA_ADDR,buf,2,true);
	
	
}

/*  
        on 값이 off 값과 비슷해질수록 V전압이 높아짐 duty
        1,0 	duty 100%   4.6V
        512,0 	duty 90%	4.1V
        1024,0 	duty 75%	
        2048,0 	duty 50%	2.3V
        3060,0 	duty 25%	1.18V
        4096,0 	duty 0%		0V
		off값 0로 고정후 on값만 변경하여 PWM 제어

*/
void setPWM(uint8_t led,uint16_t on_value){
	uint8_t wbuf[2];
	uint16_t ON_L_H=0x00,OFF_L_H=0x00;
	if(on_value>1)
	on_value--;


	wbuf[0]=PWM_ON_L+((led-1)*MULTI);
	wbuf[1]=(uint8_t)on_value;
	i2c_write_blocking(PICO_i2c1,pca_addr,wbuf,2,false);
	printf("ON_L : %02xh , D.%02x \r\n",wbuf[0],wbuf[1]);

	wbuf[0]=PWM_ON_H+((led-1)*MULTI);
	ON_L_H=on_value>>8;
	wbuf[1]=(uint8_t)ON_L_H;
	i2c_write_blocking(PICO_i2c1,pca_addr,wbuf,2,false);
	printf("ON_H : %02xh , D.%02x \r\n",wbuf[0],wbuf[1]);

	// wbuf[0]=PWM_OFF_L+((led-1)*MULTI);	
	// wbuf[1]=(uint8_t)off_value;
	// i2c_write_blocking(PICO_i2c1,pca_addr,wbuf,2,false);
	// printf("OFF_L : %02xh , D.%02x \r\n",wbuf[0],wbuf[1]);

	// wbuf[0]=PWM_OFF_H+((led-1)*MULTI);
	// OFF_L_H=off_value>>8;
	// wbuf[1]=(uint8_t)OFF_L_H;
	// i2c_write_blocking(PICO_i2c1,pca_addr,wbuf,2,false);
	// printf("OFF_H : %02xh , D.%02x \r\n",wbuf[0],wbuf[1]);
	
	printf("pwm %d Ch on:%04x , addr:%02x \r\n",led,on_value,pca_addr);
}

void setMotor(uint8_t ch,uint8_t volt)
{	
	setPWM(ch,4096-cmap(volt,0,12,0,4095));
}

void setValve(uint8_t ch,uint8_t set)
{
	setPWM(ch,set);
}

void read_PCA_reg(uint8_t reg,uint8_t len) {
	uint8_t read_data[len];
	i2c_write_blocking(PICO_i2c1, pca_addr,reg , 1, true);
	i2c_read_blocking(PICO_i2c1, pca_addr,read_data , len, false);

	for(int i=0;i<=len-1;i++)
	printf("reg:%02x / %d = %02x \r\n",reg,i,read_data[i]);
}

void write_PCA_reg(uint8_t reg,uint8_t rx){
	uint8_t rxbuf[2];

	rxbuf[0]=reg;
	rxbuf[1]=rx;
	i2c_write_blocking(PICO_i2c1, pca_addr,rxbuf , 2, false);
	printf("reg:%02x, rx:%02x",reg,rx);
}
/*
* PCA mode1 기본상태 = 0x10 으로 데이터시트상 슬립모드이다
* 0x00값을 씀으로 슬립모드를 해제하여 사용해야한다.
*
*/
void getPCAmode(uint8_t setPCA)
{
	uint8_t ret;
	uint8_t rxdata,setData[2]={PCA9685_MODE1,setPCA}; // sleep 상태에서 wake 상태로 변경
	// i2c_write_blocking(PICO_i2c1, pca_addr,setData , 2, false); // 설정 변경 확인됨
	i2c_bus_scan();	

	i2c_write_blocking(PICO_i2c1, pca_addr,setData , 2, false); // mode0에 0x00값 입력
	
	i2c_write_blocking(PICO_i2c1, pca_addr,PCA9685_MODE1 , 1, true);
	ret = i2c_read_blocking(PICO_i2c1, pca_addr,rxdata , 1, false);
	printf("0x40 : %02x\r\n",ret);
	printf("mode1 : %02x \r\n",rxdata);


	// 모든 출력 OFF값 0x0000로 설정
	setData[0]=PWMALL_OFF_L;
	setData[1]=0x00;
	i2c_write_blocking(PICO_i2c1,pca_addr,setData,2,false);
	setData[0]=PWMALL_OFF_H;
	setData[1]=0x00;
	i2c_write_blocking(PICO_i2c1,pca_addr,setData,2,false);
	
}



