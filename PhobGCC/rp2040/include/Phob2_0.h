#ifndef BOARD_H
#define BOARD_H

#include <cmath>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#include "structsAndEnums.h"
#include "storage/pages/storage.h"

#ifndef CLEANADC
#define CLEANADC
#endif //CLEANADC

const int _us = 125;

//defining which pin is what
//GPIO
//TODO: these are all temporary
const int _pinA =  1;
const int _pinB =  2;
const int _pinDr = 9;
const int _pinDu = 21;
const int _pinDl = 22;
const int _pinDd = 28;
const int _pinL =  3;
const int _pinR =  4;
const int _pinX =  5;
const int _pinY =  6;
const int _pinZ =  7;
const int _pinS =  8;
const int _pinRumble = 24;
const int _pinBrake = 14;
const int _pinTX  = 15;
//GPIO SPI for ADCs
const int _pinSPIrx = 16;
const int _pinAcs = 17;
const int _pinSPIclk = 18;
const int _pinSPItx = 19;
const int _pinCcs = 20;
//a little resistor ladder DAC
const int _pinDac0 = 10;
const int _pinDac1 = 11;
const int _pinDac2 = 12;
const int _pinDac3 = 13;
//this is only for the pico itself, not necessarily the phob
const int _pinLED = 25;
//two of the built-in ADCs:
const int _pinRa = 26; //GPIO number
const int _pinRadc = 0; //ADC number
const int _pinLa = 27; //GPIO number
const int _pinLadc = 1; //ADC number
//and two more
const int _pinSpare0 = 0;
const int _pinSpare1 = 23;
const int _pinSpare2 = 29;

const int _pinAx = -1;
const int _pinAy = -1;
const int _pinCx = -1;
const int _pinCy = -1;
const int _pinRX = -1;

void setPinModes() {
	gpio_init(_pinA);
	gpio_pull_up(_pinA);
	gpio_set_dir(_pinA, GPIO_IN);
	gpio_init(_pinB);
	gpio_pull_up(_pinB);
	gpio_set_dir(_pinB, GPIO_IN);
	gpio_init(_pinDr);
	gpio_pull_up(_pinDr);
	gpio_set_dir(_pinDr, GPIO_IN);
	gpio_init(_pinDu);
	gpio_pull_up(_pinDu);
	gpio_set_dir(_pinDu, GPIO_IN);
	gpio_init(_pinDl);
	gpio_pull_up(_pinDl);
	gpio_set_dir(_pinDl, GPIO_IN);
	gpio_init(_pinDd);
	gpio_pull_up(_pinDd);
	gpio_set_dir(_pinDd, GPIO_IN);
	gpio_init(_pinL);
	gpio_pull_up(_pinL);
	gpio_set_dir(_pinL, GPIO_IN);
	gpio_init(_pinR);
	gpio_pull_up(_pinR);
	gpio_set_dir(_pinR, GPIO_IN);
	gpio_init(_pinX);
	gpio_pull_up(_pinX);
	gpio_set_dir(_pinX, GPIO_IN);
	gpio_init(_pinY);
	gpio_pull_up(_pinY);
	gpio_set_dir(_pinY, GPIO_IN);
	gpio_init(_pinZ);
	gpio_pull_up(_pinZ);
	gpio_set_dir(_pinZ, GPIO_IN);
	gpio_init(_pinS);
	gpio_pull_up(_pinS);
	gpio_set_dir(_pinS, GPIO_IN);
	gpio_init(_pinLED);
	gpio_set_dir(_pinLED, GPIO_OUT);
	gpio_init(_pinRumble);
	gpio_set_dir(_pinRumble, GPIO_OUT);
	gpio_init(_pinBrake);
	gpio_set_dir(_pinBrake, GPIO_OUT);

	/* the comms library sets this it seems
	gpio_init(_pinTx);
	gpio_set_dir(_pinTx, GPIO_IN);
	gpio_pull_up(_pinTx);
	*/

	//initialize SPI at 1 MHz
	//initialize SPI at 3 MHz just to test
	spi_init(spi0, 3000*1000);
	gpio_set_function(_pinSPIclk, GPIO_FUNC_SPI);
	gpio_set_function(_pinSPItx, GPIO_FUNC_SPI);
	gpio_set_function(_pinSPIrx, GPIO_FUNC_SPI);
	gpio_init(_pinAcs);
	gpio_set_dir(_pinAcs, GPIO_OUT);
	gpio_put(_pinAcs, 1);//active low
	gpio_init(_pinCcs);
	gpio_set_dir(_pinCcs, GPIO_OUT);
	gpio_put(_pinCcs, 1);//active low

	//initialize ADC for triggers
	adc_init();
	adc_gpio_init(_pinLa);
	adc_gpio_init(_pinRa);

	//initialize DAC outputs
	gpio_init(_pinDac0);
	gpio_init(_pinDac1);
	gpio_init(_pinDac2);
	gpio_init(_pinDac3);
	gpio_set_dir(_pinDac0, GPIO_OUT);
	gpio_set_dir(_pinDac1, GPIO_OUT);
	gpio_set_dir(_pinDac2, GPIO_OUT);
	gpio_set_dir(_pinDac3, GPIO_OUT);

	//initialize spare outputs
	gpio_init(_pinSpare0);
	gpio_set_dir(_pinSpare0, GPIO_OUT);
}

void readButtons(const Pins &, Buttons &hardware) {
	hardware.A  = !gpio_get(_pinA);
	hardware.B  = !gpio_get(_pinB);
	hardware.X  = !gpio_get(_pinX);
	hardware.Y  = !gpio_get(_pinY);
	hardware.L  = !gpio_get(_pinL);
	hardware.R  = !gpio_get(_pinR);
	hardware.Z  = !gpio_get(_pinZ);
	hardware.S  = !gpio_get(_pinS);
	hardware.Dr = !gpio_get(_pinDr);
	hardware.Du = !gpio_get(_pinDu);
	hardware.Dl = !gpio_get(_pinDl);
	hardware.Dd = !gpio_get(_pinDd);
}

void readADCScale(float &, float ) {
	//do nothing
}

int readLa(const Pins &, const int initial, const float scale) {
	adc_select_input(_pinLadc);
	float temp = adc_read() / 16.0;
	return fmin(255, fmax(0, temp - initial) * scale);
}
int readRa(const Pins &, const int initial, const float scale) {
	adc_select_input(_pinRadc);
	float temp = adc_read() / 16.0;
	return fmin(255, fmax(0, temp - initial) * scale);
}

//for external MCP3002 adc
int __time_critical_func(readExtAdc)(const WhichStick whichStick, const WhichAxis whichAxis) {
	//                              leading zero to align read bytes
	//                              |start bit
	//                              ||absolute, two channels
	//                              |||channel 0
	//                              ||||most significant bit first
	//                              |||||(don't care, even though it gets repeated)
	uint8_t configBits[] = {0b01101000};
	if(whichAxis == YAXIS) {
		configBits[0] = 0b01111000;//channel 1
	}
	uint8_t buf[2];
	//asm volatile("nop \n nop \n nop");//these were in the example; are they needed?
	if(whichStick == ASTICK) {
		//left stick
		gpio_put(_pinAcs, 0);
	} else {
		//c-stick
		gpio_put(_pinCcs, 0);
	}
	//asm volatile("nop \n nop \n nop");

	spi_read_blocking(spi0, *configBits, buf, 2);
	//only the last two bits of the first byte are used
	uint16_t tempValue = (((buf[0] & 0b00000011) << 8) | buf[1]);
	//we want it to read out as if it's 12-bit, instead of 10-bit like it is
	tempValue = tempValue << 2;

	//asm volatile("nop \n nop \n nop");
	if(whichStick == ASTICK) {
		gpio_put(_pinAcs, 1);
	} else {
		gpio_put(_pinCcs, 1);
	}
	//asm volatile("nop \n nop \n nop");

	return tempValue;
}

//For MCP3002 ADC
int readAx(const Pins &) {
	return readExtAdc(ASTICK, XAXIS);
}
int readAy(const Pins &) {
	return readExtAdc(ASTICK, YAXIS);
}
int readCx(const Pins &) {
	return readExtAdc(CSTICK, XAXIS);
}
int readCy(const Pins &) {
	return readExtAdc(CSTICK, YAXIS);
}

uint64_t micros() {
	return time_us_64();
}

uint64_t millis() {
	return time_us_64()/1000;
}

#endif //BOARD_H
