#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "phobGCC.h"
#include "comms/joybus.hpp"

//This gets called by the comms library
GCReport buttonsToGCReport() {
	GCReport report = {
		.a       = _btn.A,
		.b       = _btn.B,
		.x       = _btn.X,
		.y       = _btn.Y,
		.start   = _btn.S,
		.pad0    = 0,
		.dLeft   = _btn.Dl,
		.dRight  = _btn.Dr,
		.dDown   = _btn.Dd,
		.dUp     = _btn.Du,
		.z       = _btn.Z,
		.r       = _btn.L,
		.l       = _btn.R,
		.pad1    = 1,
		.xStick  = _btn.Ax,
		.yStick  = _btn.Ay,
		.cxStick = _btn.Cx,
		.cyStick = _btn.Cy,
		.analogL = _btn.La,
		.analogR = _btn.Ra
	};
	return report;
}

void second_core() {
	const int numberOfNaN = readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams);

	if(numberOfNaN > 10){//by default it seems 16 end up unitialized on pico
		resetDefaults(FACTORY, _controls, _gains, _normGains, _aStickParams, _cStickParams);
		readEEPROM(_controls, _gains, _normGains, _aStickParams, _cStickParams);
	}

	gpio_set_function(_pinLED, GPIO_FUNC_PWM);

	uint slice_num = pwm_gpio_to_slice_num(_pinLED);

	pwm_set_wrap(slice_num, 255);
	pwm_set_chan_level(slice_num, PWM_CHAN_B, 128);
	pwm_set_enabled(slice_num, true);


	while(true) {
		sleep_ms(4000);
		pwm_set_gpio_level(_pinLED, 63);
		sleep_ms(250);
		pwm_set_gpio_level(_pinLED, 127);
		sleep_ms(250);
		pwm_set_gpio_level(_pinLED, 191);
		sleep_ms(250);
		pwm_set_gpio_level(_pinLED, 255);

		sleep_ms(4000);
		for(int i=0; i<numberOfNaN; i++) {
			pwm_set_gpio_level(_pinLED, 0);
			sleep_ms(500);
			pwm_set_gpio_level(_pinLED, 255);
			sleep_ms(500);
		}
		pwm_set_gpio_level(_pinLED, 0);
		sleep_ms(4000);
		if(numberOfNaN == 0) {
			pwm_set_gpio_level(_pinLED, 0);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 255);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 0);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 255);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 0);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 255);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 0);
			sleep_ms(100);
			pwm_set_gpio_level(_pinLED, 255);
			sleep_ms(100);
		}
		pwm_set_gpio_level(_pinLED, 0);
	}


}

int main() {
	//set the clock speed to 125 kHz
	//the comms library needs this clockspeed
	set_sys_clock_khz(1000*_us, true);

	//set up the main core so it can be paused by the other core
	//this is necessary for flash writing
	multicore_lockout_victim_init();

	setPinModes();

	Pins pinList {
		.pinLa = 0,
		.pinRa = 0,
		.pinL  = 0,
		.pinR  = 0,
		.pinAx = 0,
		.pinAy = 0,
		.pinCx = 0,
		.pinCy = 0,
		.pinRX = 0,
		.pinTX = 0,
		.pinDr = 0,
		.pinDu = 0,
		.pinDl = 0,
		.pinDd = 0,
		.pinX  = 0,
		.pinY  = 0,
		.pinA  = 0,
		.pinB  = 0,
		.pinZ  = 0,
		.pinS  = 0
	};

	_btn.A      =0;
	_btn.B      =0;
	_btn.X      =0;
	_btn.Y      =0;
	_btn.S      =0;
	_btn.orig   =0;
	_btn.errL   =0;
	_btn.errS   =0;
	_btn.Dl     =0;
	_btn.Dr     =0;
	_btn.Dd     =0;
	_btn.Du     =0;
	_btn.Z      =0;
	_btn.R      =0;
	_btn.L      =0;
	_btn.high   =1;
	_btn.Ax     =127;
	_btn.Ay     =127;
	_btn.Cx     =127;
	_btn.Cy     =127;
	_btn.La     =0;
	_btn.Ra     =0;
	_btn.magic1 =0;
	_btn.magic2 =0;

	multicore_launch_core1(second_core);

	// ADC TEST
	/*
	gpio_set_function(_pinLED, GPIO_FUNC_PWM);

	uint slice_num = pwm_gpio_to_slice_num(_pinLED);

	pwm_set_wrap(slice_num, 255);
	pwm_set_chan_level(slice_num, PWM_CHAN_B, 128);
	pwm_set_enabled(slice_num, true);

	while(true) {
		sleep_ms(4000);
		pwm_set_gpio_level(_pinLED, 63);
		sleep_ms(250);
		pwm_set_gpio_level(_pinLED, 127);
		sleep_ms(250);
		pwm_set_gpio_level(_pinLED, 191);
		sleep_ms(250);
		pwm_set_gpio_level(_pinLED, 255);

		int Ax = readAx(pinList) / 17;
		int Ay = readAy(pinList) / 17;
		int Cx = readCx(pinList) / 17;
		int Cy = readCy(pinList) / 17;
		sleep_ms(4000);
		pwm_set_gpio_level(_pinLED, Ax);
		sleep_ms(1000);
		pwm_set_gpio_level(_pinLED, Ay);
		sleep_ms(1000);
		pwm_set_gpio_level(_pinLED, Cx);
		sleep_ms(1000);
		pwm_set_gpio_level(_pinLED, Cy);
	}
	*/

	//Run comms
	enterMode(_pinTX, buttonsToGCReport);
}
