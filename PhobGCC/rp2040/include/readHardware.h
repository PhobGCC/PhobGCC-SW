#ifndef READHARDWARE_H
#define READHARDWARE_H

//include this after the pins are defined in the RP2040 board-specific headers

#include <cmath>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/timer.h"

#include "structsAndEnums.h"
#include "storage/pages/storage.h"

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

	/* the comms library sets this
	gpio_init(_pinTx);
	gpio_set_dir(_pinTx, GPIO_IN);
	gpio_pull_up(_pinTx);
	*/

	//Rumble
    gpio_init(_pinRumble);
    gpio_init(_pinBrake);
    gpio_set_dir(_pinRumble, GPIO_OUT);
    gpio_set_dir(_pinBrake, GPIO_OUT);
    gpio_set_function(_pinRumble, GPIO_FUNC_PWM);
    gpio_set_function(_pinBrake,  GPIO_FUNC_PWM);
    const uint rumbleSlice_num = pwm_gpio_to_slice_num(_pinRumble);
    const uint brakeSlice_num  = pwm_gpio_to_slice_num(_pinBrake);
    pwm_set_wrap(rumbleSlice_num, 255);
    pwm_set_wrap(brakeSlice_num,  255);
    pwm_set_chan_level(rumbleSlice_num, PWM_CHAN_B, 0);//B for odd pins
    pwm_set_chan_level(brakeSlice_num,  PWM_CHAN_B, 255);//B for odd pins
    pwm_set_enabled(rumbleSlice_num, true);
    pwm_set_enabled(brakeSlice_num,  true);

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
	gpio_init(_pinSpare1);
	gpio_set_dir(_pinSpare1, GPIO_OUT);
	gpio_init(_pinSpare2);
	gpio_set_dir(_pinSpare2, GPIO_OUT);
	gpio_init(_pinLED);
	gpio_set_dir(_pinLED, GPIO_OUT);
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

//implement a 3 unit deadzone
int readLa(const Pins &, const int initial, const float scale) {
	adc_select_input(_pinLadc);
	float temp = adc_read() / 16.0;
	if(temp < 3) {
		temp = 0.0f;
	}
	return fmin(255, fmax(0, temp - initial) * scale);
}
int readRa(const Pins &, const int initial, const float scale) {
	adc_select_input(_pinRadc);
	float temp = adc_read() / 16.0;
	if(temp < 3) {
		temp = 0.0f;
	}
	return fmin(255, fmax(0, temp - initial) * scale);
}

/*
//for external MCP3002 adc, 10 bit
int __time_critical_func(readExtAdc)(const WhichStick whichStick, const WhichAxis whichAxis) {
	//                        leading zero to align read bytes
	//                        |start bit
	//                        ||absolute, two channels
	//                        |||channel 0
	//                        ||||most significant bit first
	//                        |||||(don't care, even though it gets repeated)
	uint8_t configBits[] = {0b01101000};
	if(whichAxis == YAXIS) {
		configBits[0] =     0b01111000;//channel 1
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
*/
//for external MCP3202 adc, 12 bit
int __time_critical_func(readExtAdc)(const WhichStick whichStick, const WhichAxis whichAxis) {
	//                        start bit
	//                        |absolute, two channels
	//                        ||channel 0
	//                        |||most significant bit first
	//                        ||||(don't care, even though it gets repeated)
	//                        ||||null bit
	//                        |||||11
	//                        ||||||10  byte 1   byte 2 (when read out)
	//                        |||||||9  87654321 0_______
	uint8_t configBits[] = {0b11010000};
	if(whichAxis == YAXIS) {
		configBits[0] =     0b11110000;//channel 1
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

	spi_read_blocking(spi0, *configBits, buf, 3);
	uint16_t tempValue = (((buf[0] & 0b00000111) << 9) | buf[1] << 1 | buf[2] >> 7);

	//asm volatile("nop \n nop \n nop");
	if(whichStick == ASTICK) {
		gpio_put(_pinAcs, 1);
	} else {
		gpio_put(_pinCcs, 1);
	}
	//asm volatile("nop \n nop \n nop");

	return tempValue;
}

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

uint32_t micros() {
	return time_us_64();
}

uint64_t millis() {
	return time_us_64()/1000;
}

#endif //READHARDWARE_H
