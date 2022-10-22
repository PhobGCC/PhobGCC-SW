#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"

#include "Phob2_0.h"

int main() {
	set_sys_clock_khz(1000*_us, true);

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
}
