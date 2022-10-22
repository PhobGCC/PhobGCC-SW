#include "pico/stdlib.h"
#include "hardware/timer.h"

#include "phobGCC.h"

int main() {
	set_sys_clock_khz(1000*_us, true);

	gpio_init(_pinLED);
	gpio_set_dir(_pinLED, GPIO_OUT);
	gpio_put(_pinLED, 1);
}
