#include "comms/joybus.hpp"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "hardware/pio.h"
#include "joybus.pio.h"
#include <string.h>
#include <math.h>

#define ORG 127

/* PIOs are separate state machines for handling IOs with high timing precision. You load a program into them and they do their stuff on their own with deterministic timing,
   communicating with the main cores via FIFOs (and interrupts, if you want).

   Attempting to bit-bang the Joybus protocol by writing down timer snapshots upon GPIO changes and waiting on timers to trigger GPIO changes is bound to encounter some
   issues related to the uncertainty of timings (the STM32F0 is pipelined). Previous versions of this used to do that and would occasionally have the controller disconnect
   for a frame due to a polling fault, which would manifest particularly on the CSS by the hand teleporting back to its default position.
   Ever since the migration to PIOs - the proper tool for the job - this problem is gone.

   This project is adapted from the communication module of pico-rectangle (https://github.com/JulienBernard3383279/pico-rectangle), a digital controller firmware.

   The PIO program expects the system clock to be 125MHz, but you can adapt it by fiddling with the delays in the PIO program.

   One PIO here is configured to wait for new bytes over the joybus protocol. The main core spins a byte to come from the PIO FIFO, which will be the Joybus command
   from the console. It then matches that byte to know whether this is probe, origin or poll request.
   Once it has matched a command and has let the console finish talking, it will start replying, but in the case of the poll command, it will first build the state.
   The state is built by calling a callback passed as parameter to enterMode. This is fine for digital controllers because it takes few microseconds, but isn't
   if your state building is going to take any longer. (The console is fine with some delay between the poll command and response, but adapters don't tolerate more than
   few microseconds) In that case, you'll need to change the control flow for the passed callback to not do any computational work itself.

   Check the T O D O s in this file for things you should check when adapting it to your project.

   I advise checking the RP2040 documentation and this video https://www.youtube.com/watch?v=yYnQYF_Xa8g&ab_channel=stacksmashing to understand
*/

void __time_critical_func(convertToPio)(const uint8_t* command, const int len, uint32_t* result, int& resultLen) {
    // PIO Shifts to the right by default
    // In: pushes batches of 8 shifted left, i.e we get [0x40, 0x03, rumble (the end bit is never pushed)]
    // Out: We push commands for a right shift with an enable pin, ie 5 (101) would be 0b11'10'11
    // So in doesn't need post processing but out does
    if (len == 0) {
        resultLen = 0;
        return;
    }
    resultLen = len/2 + 1;
    int i;
    for (i = 0; i < resultLen; i++) {
        result[i] = 0;
    }
    for (i = 0; i < len; i++) {
        for (int j = 0; j < 8; j++) {
            result[i / 2] += 1 << (2 * (8 * (i % 2) + j) + 1);
            result[i / 2] += (!!(command[i] & (0x80u >> j))) << (2 * (8 * (i % 2) + j));
        }
    }
    // End bit
    result[len / 2] += 3 << (2 * (8 * (len % 2)));
}

void __time_critical_func(convertGCReport)(GCReport* report, GCReport* dest_report, uint8_t mode) {
    memcpy(dest_report, report, 8);
    if (mode == 1) {
        dest_report->mode1.cxStick = report->cxStick >> 4;
        dest_report->mode1.cyStick = report->cyStick >> 4;
        dest_report->mode1.analogL = report->analogL;
        dest_report->mode1.analogR = report->analogR;
        dest_report->mode1.analogA = 0;
        dest_report->mode1.analogB = 0;
    }
    else if (mode == 2) {
        dest_report->mode2.cxStick = report->cxStick >> 4;
        dest_report->mode2.cyStick = report->cyStick >> 4;
        dest_report->mode2.analogL = report->analogL >> 4;
        dest_report->mode2.analogR = report->analogR >> 4;
        dest_report->mode2.analogA = 0;
        dest_report->mode2.analogB = 0;
    }
    else if (mode == 4) {
        dest_report->mode4.cxStick = report->cxStick;
        dest_report->mode4.cyStick = report->cyStick;
        dest_report->mode4.analogA = 0;
        dest_report->mode4.analogB = 0;
    }
    else if (mode == 3) {
        return;
    }
    else { // Mode 0, 5, 6, 7
        dest_report->mode0.cxStick = report->cxStick;
        dest_report->mode0.cyStick = report->cyStick;
        dest_report->mode0.analogL = report->analogL >> 4;
        dest_report->mode0.analogR = report->analogR >> 4;
        dest_report->mode0.analogA = 0;
        dest_report->mode0.analogB = 0;
    }
}

void __time_critical_func(enterMode)(const int dataPin,
                                     const int rumblePin,
                                     const int brakePin,
                                     int &rumblePower,
                                     std::function<GCReport()> func) {
    gpio_init(dataPin);
    gpio_set_dir(dataPin, GPIO_IN);
    gpio_pull_up(dataPin);

    sleep_us(100); // Stabilize voltages

    PIO pio = pio0;
    pio_gpio_init(pio, dataPin);
    uint offset = pio_add_program(pio, &joybus_program);

    pio_sm_config config = joybus_program_get_default_config(offset);
    sm_config_set_in_pins(&config, dataPin);
    sm_config_set_out_pins(&config, dataPin, 1);
    sm_config_set_set_pins(&config, dataPin, 1);
    sm_config_set_clkdiv(&config, 5);
    sm_config_set_out_shift(&config, true, false, 32);
    sm_config_set_in_shift(&config, false, true, 8);
    
    pio_sm_init(pio, 0, offset, &config);
    pio_sm_set_enabled(pio, 0, true);
    
    while (true) {
		uint8_t joybusByte = pio_sm_get_blocking(pio, 0);

        if (joybusByte == 0) { // Probe
            uint8_t probeResponse[3] = { 0x09, 0x00, 0x03 };
            uint32_t result[2];
            int resultLen;
            convertToPio(probeResponse, 3, result, resultLen);
            sleep_us(6); // 3.75us into the bit before end bit => 6.25 to wait if the end-bit is 5us long

            pio_sm_set_enabled(pio, 0, false);
            pio_sm_init(pio, 0, offset+joybus_offset_outmode, &config);
            pio_sm_set_enabled(pio, 0, true);

            for (int i = 0; i<resultLen; i++) pio_sm_put_blocking(pio, 0, result[i]);
        }
        else if (joybusByte == 0x41) { // Origin (NOT 0x81)
            gpio_put(25, 1);
            uint8_t originResponse[10] = { 0x00, 0x80, ORG, ORG, ORG, ORG, 0, 0, 0, 0 };
            // TODO The origin response sends centered values in this code excerpt. Consider whether that makes sense for your project (digital controllers -> yes)
            uint32_t result[6];
            int resultLen;
            convertToPio(originResponse, 10, result, resultLen);
            // Here we don't wait because convertToPio takes time

            pio_sm_set_enabled(pio, 0, false);
            pio_sm_init(pio, 0, offset+joybus_offset_outmode, &config);
            pio_sm_set_enabled(pio, 0, true);

            for (int i = 0; i<resultLen; i++) pio_sm_put_blocking(pio, 0, result[i]);
        }
        else if (joybusByte == 0x40) { // Could check values past the first byte for reliability
            //The call to the state building function happens here, because on digital controllers, it's near instant, so it can be done between the poll and the response
            // It must be very fast (few us max) to be done between poll and response and still be compatible with adapters
            // Consider whether that makes sense for your project. If your state building is long, use a different control flow i.e precompute somehow and have func read it
            GCReport gcReport = func();
            GCReport dest_report;

			//get the second byte; we do this interleaved with work that must be done
            joybusByte = pio_sm_get_blocking(pio, 0);

            convertGCReport(&gcReport, &dest_report, joybusByte);

            uint32_t result[5];
            int resultLen;
            convertToPio((uint8_t*)(&dest_report), 8, result, resultLen);

			//get the third byte; we do this interleaved with work that must be done
            joybusByte = pio_sm_get_blocking(pio, 0);

			//sleep_us(4);//add delay so we don't overwrite the stop bit
			sleep_us(7);//add delay so we don't overwrite the stop bit

            pio_sm_set_enabled(pio, 0, false);
            pio_sm_init(pio, 0, offset+joybus_offset_outmode, &config);
            pio_sm_set_enabled(pio, 0, true);

            for (int i = 0; i<resultLen; i++) pio_sm_put_blocking(pio, 0, result[i]);

			//Rumble
			bool rumbleBrake = joybusByte & 2;
			bool rumble = (joybusByte & 1) && !rumbleBrake;
            if(rumble) {
                pwm_set_gpio_level(brakePin, 0);
                pwm_set_gpio_level(rumblePin, rumblePower);
            } else {
                pwm_set_gpio_level(rumblePin, 0);
                pwm_set_gpio_level(brakePin, rumbleBrake ? 255 : 0);
            }

        }
        else {
            pio_sm_set_enabled(pio, 0, false);
            sleep_us(400);
            //If an unmatched communication happens, we wait for 400us for it to finish for sure before starting to listen again
            pio_sm_init(pio, 0, offset+joybus_offset_inmode, &config);
            pio_sm_set_enabled(pio, 0, true);
        }
    }
}

