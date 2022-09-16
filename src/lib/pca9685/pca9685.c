

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pca9685.h"
#include "DEV_Config.h"

// Setup registers
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

// Define first LED and all LED. We calculate the rest
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA
#define PCA_ADDR 0x40<<1
#define PIN_ALL 16


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

	i2c_init(PICO_i2c1, 100 * 1000);
    gpio_set_function(PICO_I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C1_SDA_PIN);
    gpio_pull_up(PICO_I2C1_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_I2C1_SDA_PIN, PICO_I2C1_SCL_PIN, GPIO_FUNC_I2C));
	Driver_Delay_ms(200);
	// setPWMFreq(4096);
	i2c_bus_scan();
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
    return 0;

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

void setPWM(uint8_t led,int on_value,int off_value){
	uint8_t wbuf[2];
	uint8_t addr=PCA_ADDR | 0x80 | 0x00;
	wbuf[0]=PWM_ON_L+((led-1)*MULTI);
	wbuf[1]=(uint8_t)on_value;
	i2c_write_blocking(PICO_i2c1,addr,wbuf,2,false);

	wbuf[0]=PWM_ON_H+((led-1)*MULTI);
	wbuf[1]=(uint8_t)on_value>>8;
	i2c_write_blocking(PICO_i2c1,addr,wbuf,2,false);

	wbuf[0]=PWM_OFF_L+((led-1)*MULTI);
	wbuf[1]=(uint8_t)off_value;
	i2c_write_blocking(PICO_i2c1,addr,wbuf,2,false);

	wbuf[0]=PWM_OFF_H+((led-1)*MULTI);
	wbuf[1]=(uint8_t)off_value>>8;
	i2c_write_blocking(PICO_i2c1,addr,wbuf,2,false);
	printf("pwm %d Ch on:%d , off:%d \r\n",led,on_value,off_value);
}

void getPCAmode()
{
	uint8_t ret;
	uint8_t rxdata[8];
	uint8_t txdata;
	uint8_t addr = PCA_ADDR ||
	txdata=0x00;
	i2c_write_blocking(PICO_i2c1, PCA_ADDR,txdata , 1, true);
	i2c_read_blocking(PICO_i2c1, PCA_ADDR,&rxdata , 1, false);
	// printf("rx hex : %.2x %.2x %.2x %.2x %.2x %.2x %.2x %.2x \r\n",rxdata[0],rxdata[1],rxdata[2],rxdata[3],rxdata[4],rxdata[5],rxdata[6],rxdata[7]);
	
	printf("rx hex : %02x\r\n",rxdata);
}
// int pca9685Setup(const int pinBase, const int i2cAddress, float freq)
// {
// 	// Create a node with 16 pins [0..15] + [16] for all
// 	struct wiringPiNodeStruct *node = wiringPiNewNode(pinBase, PIN_ALL + 1);

// 	// Check if pinBase is available
// 	if (!node)
// 		return -1;

// 	// Check i2c address
// 	int fd = wiringPiI2CSetup(i2cAddress);
// 	if (fd < 0)
// 		return fd;

// 	// Setup the chip. Enable auto-increment of registers.
// 	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
// 	int autoInc = settings | 0x20;

// 	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, autoInc);
	
// 	// Set frequency of PWM signals. Also ends sleep mode and starts PWM output.
// 	if (freq > 0)
// 		pca9685PWMFreq(fd, freq);
	

// 	node->fd			= fd;
// 	node->pwmWrite		= myPwmWrite;
// 	node->digitalWrite	= myOnOffWrite;
// 	node->digitalRead	= myOffRead;
// 	node->analogRead	= myOnRead;

// 	return fd;
// }

// /**
//  * Sets the frequency of PWM signals.
//  * Frequency will be capped to range [40..1000] Hertz. Try 50 for servos.
//  */
// void pca9685PWMFreq(int fd, float freq)
// {
// 	// Cap at min and max
// 	freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));

// 	// To set pwm frequency we have to set the prescale register. The formula is:
// 	// prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
// 	// Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
// 	int prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);

// 	// Get settings and calc bytes for the different states.
// 	int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;	// Set restart bit to 0
// 	int sleep	= settings | 0x10;									// Set sleep bit to 1
// 	int wake 	= settings & 0xEF;									// Set sleep bit to 0
// 	int restart = wake | 0x80;										// Set restart bit to 1

// // 	// Go to sleep, set prescale and wake up again.
// 	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);
// 	wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);
// 	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);

// // 	// Now wait a millisecond until oscillator finished stabilizing and restart PWM.
// 	delay(1);
// 	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart);
// }

// /**
//  * Set all leds back to default values (: fullOff = 1)
//  */
// void pca9685PWMReset(int fd)
// {
// 	wiringPiI2CWriteReg16(fd, LEDALL_ON_L	 , 0x0);
// 	wiringPiI2CWriteReg16(fd, LEDALL_ON_L + 2, 0x1000);
// }

// /**
//  * Write on and off ticks manually to a pin
//  * (Deactivates any full-on and full-off)
//  */
// void pca9685PWMWrite(int fd, int pin, int on, int off)
// {
// 	int reg = baseReg(pin);

// 	// Write to on and off registers and mask the 12 lowest bits of data to overwrite full-on and off
// 	wiringPiI2CWriteReg16(fd, reg	 , on  & 0x0FFF);
// 	wiringPiI2CWriteReg16(fd, reg + 2, off & 0x0FFF);
// }

// /**
//  * Reads both on and off registers as 16 bit of data
//  * To get PWM: mask each value with 0xFFF
//  * To get full-on or off bit: mask with 0x1000
//  * Note: ALL_LED pin will always return 0
//  */
// void pca9685PWMRead(int fd, int pin, int *on, int *off)
// {
// 	int reg = baseReg(pin);

// 	if (on)
// 		*on  = wiringPiI2CReadReg16(fd, reg);
// 	if (off)
// 		*off = wiringPiI2CReadReg16(fd, reg + 2);
// }

// /**
//  * Enables or deactivates full-on
//  * tf = true: full-on
//  * tf = false: according to PWM
//  */
// void pca9685FullOn(int fd, int pin, int tf)
// {
// 	int reg = baseReg(pin) + 1;		// LEDX_ON_H
// 	int state = wiringPiI2CReadReg8(fd, reg);

// 	// Set bit 4 to 1 or 0 accordingly
// 	state = tf ? (state | 0x10) : (state & 0xEF);

// 	wiringPiI2CWriteReg8(fd, reg, state);

// 	// For simplicity, we set full-off to 0 because it has priority over full-on
// 	if (tf)
// 		pca9685FullOff(fd, pin, 0);
// }

// /**
//  * Enables or deactivates full-off
//  * tf = true: full-off
//  * tf = false: according to PWM or full-on
//  */
// void pca9685FullOff(int fd, int pin, int tf)
// {
// 	int reg = baseReg(pin) + 3;		// LEDX_OFF_H
// 	int state = wiringPiI2CReadReg8(fd, reg);

// 	// Set bit 4 to 1 or 0 accordingly
// 	state = tf ? (state | 0x10) : (state & 0xEF);

// 	wiringPiI2CWriteReg8(fd, reg, state);
// }

// /**
//  * Helper function to get to register
//  */
// int baseReg(int pin)
// {
// 	return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
// }




// //------------------------------------------------------------------------------------------------------------------
// //
// //	WiringPi functions
// //
// //------------------------------------------------------------------------------------------------------------------




// /**
//  * Simple PWM control which sets on-tick to 0 and off-tick to value.
//  * If value is <= 0, full-off will be enabled
//  * If value is >= 4096, full-on will be enabled
//  * Every value in between enables PWM output
//  */
// static void myPwmWrite(struct wiringPiNodeStruct *node, int pin, int value)
// {
// 	int fd   = node->fd;
// 	int ipin = pin - node->pinBase;

// 	if (value >= 4096)
// 		pca9685FullOn(fd, ipin, 1);
// 	else if (value > 0)
// 		pca9685PWMWrite(fd, ipin, 0, value);	// (Deactivates full-on and off by itself)
// 	else
// 		pca9685FullOff(fd, ipin, 1);
// }

// /**
//  * Simple full-on and full-off control
//  * If value is 0, full-off will be enabled
//  * If value is not 0, full-on will be enabled
//  */
// static void myOnOffWrite(struct wiringPiNodeStruct *node, int pin, int value)
// {
// 	int fd   = node->fd;
// 	int ipin = pin - node->pinBase;

// 	if (value)
// 		pca9685FullOn(fd, ipin, 1);
// 	else
// 		pca9685FullOff(fd, ipin, 1);
// }

// /**
//  * Reads off registers as 16 bit of data
//  * To get PWM: mask with 0xFFF
//  * To get full-off bit: mask with 0x1000
//  * Note: ALL_LED pin will always return 0
//  */
// static int myOffRead(struct wiringPiNodeStruct *node, int pin)
// {
// 	int fd   = node->fd;
// 	int ipin = pin - node->pinBase;

// 	int off;
// 	pca9685PWMRead(fd, ipin, 0, &off);

// 	return off;
// }

// /**
//  * Reads on registers as 16 bit of data
//  * To get PWM: mask with 0xFFF
//  * To get full-on bit: mask with 0x1000
//  * Note: ALL_LED pin will always return 0
//  */
// static int myOnRead(struct wiringPiNodeStruct *node, int pin)
// {
// 	int fd   = node->fd;
// 	int ipin = pin - node->pinBase;

// 	int on;
// 	pca9685PWMRead(fd, ipin, &on, 0);

// 	return on;
// }






