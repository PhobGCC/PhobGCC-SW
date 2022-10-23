#ifndef BOARD_H
#define BOARD_H

#include <cmath>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"

#include "structsAndEnums.h"
#include "storage/pages/storage.h"

const int _us = 125;

//defining which pin is what
//GPIO
//TODO: these are all temporary
const int _pinA =  1;
const int _pinB =  2;
const int _pinDr = 3;
const int _pinDu = 4;
const int _pinDl = 5;
const int _pinDd = 6;
const int _pinL =  7;
const int _pinR =  8;
const int _pinX =  9;
const int _pinY =  10;
const int _pinZ =  11;
const int _pinS =  12;
const int _pinRumble = 13;
const int _pinBrake = 14;
const int _pinTX  = 15;
//GPIO SPI for ADCs
const int _pinSPIrx = 16;
const int _pinAcs = 17;
const int _pinSPIclk = 18;
const int _pinSPItx = 19;
const int _pinCcs = 20;
//a little resistor ladder DAC
const int _pinDac0 = 21;
const int _pinDac1 = 22;
const int _pinDac2 = 23;
const int _pinDac3 = 24;
//this is only for the pico itself, not necessarily the phob
const int _pinLED = 25;
//two of the built-in ADCs:
const int _pinRa = 26; //GPIO number
const int _pinRadc = 0; //ADC number
const int _pinLa = 27; //GPIO number
const int _pinLadc = 1; //ADC number
//and two more
const int _pinSpare0 = 28;
const int _pinSpare1 = 29;

const int _pinAx = -1;
const int _pinAy = -1;
const int _pinCx = -1;
const int _pinCy = -1;
const int _pinRX = -1;

void setPinModes() {
	gpio_init(_pinA);
	gpio_set_dir(_pinA, GPIO_IN);
	gpio_init(_pinB);
	gpio_set_dir(_pinB, GPIO_IN);
	gpio_init(_pinDr);
	gpio_set_dir(_pinDr, GPIO_IN);
	gpio_init(_pinDu);
	gpio_set_dir(_pinDu, GPIO_IN);
	gpio_init(_pinDl);
	gpio_set_dir(_pinDl, GPIO_IN);
	gpio_init(_pinDd);
	gpio_set_dir(_pinDd, GPIO_IN);
	gpio_init(_pinL);
	gpio_set_dir(_pinL, GPIO_IN);
	gpio_init(_pinR);
	gpio_set_dir(_pinR, GPIO_IN);
	gpio_init(_pinX);
	gpio_set_dir(_pinX, GPIO_IN);
	gpio_init(_pinY);
	gpio_set_dir(_pinY, GPIO_IN);
	gpio_init(_pinZ);
	gpio_set_dir(_pinZ, GPIO_IN);
	gpio_init(_pinS);
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
	spi_init(spi0, 1000*1000);
	gpio_set_function(_pinSPIclk, GPIO_FUNC_SPI);
	gpio_set_function(_pinSPItx, GPIO_FUNC_SPI);
	gpio_set_function(_pinSPIrx, GPIO_FUNC_SPI);
	gpio_init(_pinAcs);
	gpio_set_dir(_pinAcs, GPIO_OUT);
	gpio_put(_pinAcs, 1);//active low
	gpio_init(_pinCcs);
	gpio_set_dir(_pinCcs, GPIO_OUT);
	gpio_put(_pinCcs, 1);//active low

	adc_init();
	adc_gpio_init(_pinLa);
	adc_gpio_init(_pinRa);
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

//For MCP3002 ADC
int readAx(const Pins &) {
	//                              leading zero to align read bytes
	//                              |start bit
	//                              ||absolute, two channels
	//                              |||channel 0
	//                              ||||most significant bit first
	//                              |||||(don't care, even though it gets repeated)
	const uint8_t configBits[] = {0b01101000};
	uint8_t buf[2];
	asm volatile("nop \n nop \n nop");
	//left stick
	gpio_put(_pinAcs, 0);
	asm volatile("nop \n nop \n nop");

	spi_read_blocking(spi0, *configBits, buf, 2);
	//only the last two bits of the first byte are used
	uint16_t tempValue = (((buf[0] & 0b00000011) << 8) | buf[1]);
	//we want it to read out as if it's 12-bit, instead of 10-bit like it is
	tempValue = tempValue << 2;

	asm volatile("nop \n nop \n nop");
	gpio_put(_pinAcs, 1);
	asm volatile("nop \n nop \n nop");

	return tempValue;
}
int readAy(const Pins &) {
	//                              leading zero to align read bytes
	//                              |start bit
	//                              ||absolute, two channels
	//                              |||channel 1
	//                              ||||most significant bit first
	//                              |||||(don't care, even though it gets repeated)
	const uint8_t configBits[] = {0b01111000};
	uint8_t buf[2];
	asm volatile("nop \n nop \n nop");
	//left stick
	gpio_put(_pinAcs, 0);
	asm volatile("nop \n nop \n nop");

	spi_read_blocking(spi0, *configBits, buf, 2);
	//only the last two bits of the first byte are used
	uint16_t tempValue = (((buf[0] & 0b00000011) << 8) | buf[1]);
	//we want it to read out as if it's 12-bit, instead of 10-bit like it is
	tempValue = tempValue << 2;

	asm volatile("nop \n nop \n nop");
	gpio_put(_pinAcs, 1);
	asm volatile("nop \n nop \n nop");

	return tempValue;
}
int readCx(const Pins &) {
	//                              leading zero to align read bytes
	//                              |start bit
	//                              ||absolute, two channels
	//                              |||channel 0
	//                              ||||most significant bit first
	//                              |||||(don't care, even though it gets repeated)
	const uint8_t configBits[] = {0b01101000};
	uint8_t buf[2];
	asm volatile("nop \n nop \n nop");
	//right stick
	gpio_put(_pinCcs, 0);
	asm volatile("nop \n nop \n nop");

	spi_read_blocking(spi0, *configBits, buf, 2);
	//only the last two bits of the first byte are used
	uint16_t tempValue = (((buf[0] & 0b00000011) << 8) | buf[1]);
	//we want it to read out as if it's 12-bit, instead of 10-bit like it is
	tempValue = tempValue << 2;

	asm volatile("nop \n nop \n nop");
	gpio_put(_pinCcs, 1);
	asm volatile("nop \n nop \n nop");

	return tempValue;
}
int readCy(const Pins &) {
	//                              leading zero to align read bytes
	//                              |start bit
	//                              ||absolute, two channels
	//                              |||channel 1
	//                              ||||most significant bit first
	//                              |||||(don't care, even though it gets repeated)
	const uint8_t configBits[] = {0b01111000};
	uint8_t buf[2];
	asm volatile("nop \n nop \n nop");
	//right stick
	gpio_put(_pinCcs, 0);
	asm volatile("nop \n nop \n nop");

	//                left stick
	spi_read_blocking(spi0, *configBits, buf, 2);
	//only the last two bits of the first byte are used
	uint16_t tempValue = (((buf[0] & 0b00000011) << 8) | buf[1]);
	//we want it to read out as if it's 12-bit, instead of 10-bit like it is
	tempValue = tempValue << 2;

	asm volatile("nop \n nop \n nop");
	gpio_put(_pinCcs, 1);
	asm volatile("nop \n nop \n nop");

	return tempValue;
}

#endif //BOARD_H
