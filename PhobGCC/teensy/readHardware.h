#ifndef READHARDWARE_H
#define READHARDWARE_H

//include this after the pins are defined in the Teensy board-specific headers

#include "../common/structsAndEnums.h"

void readButtons(Buttons &btn, HardwareButtons &hardware, ControlConfig &controls) {
	btn.A = !digitalRead(_pinA);
	btn.B = !digitalRead(_pinB);
	btn.X = !digitalRead(controls.pinXSwappable);
	btn.Y = !digitalRead(controls.pinYSwappable);
	btn.Z = !digitalRead(controls.pinZSwappable);
	btn.S = !digitalRead(_pinS);
	btn.Du = !digitalRead(_pinDu);
	btn.Dd = !digitalRead(_pinDd);
	btn.Dl = !digitalRead(_pinDl);
	btn.Dr = !digitalRead(_pinDr);

	hardware.L = !digitalRead(_pinL);
	hardware.R = !digitalRead(_pinR);
	hardware.Z = !digitalRead(_pinZ);
	hardware.X = !digitalRead(_pinX);
	hardware.Y = !digitalRead(_pinY);
}

#endif //READHARDWARE_H
