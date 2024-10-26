#ifndef READHARDWARE_H
#define READHARDWARE_H

//include this after the pins are defined in the Teensy board-specific headers

#include <ADC.h>
#include <VREF.h>
#include "../common/structsAndEnums.h"

ADC *adc = new ADC();

void setPinModes(){
	pinMode(_pinL,INPUT_PULLUP);
	pinMode(_pinR,INPUT_PULLUP);
	pinMode(_pinDr,INPUT_PULLUP);
	pinMode(_pinDu,INPUT_PULLUP);
	pinMode(_pinDl,INPUT_PULLUP);
	pinMode(_pinDd,INPUT_PULLUP);
	pinMode(_pinX,INPUT_PULLUP);
	pinMode(_pinY,INPUT_PULLUP);

	pinMode(_pinA,INPUT_PULLUP);
	pinMode(_pinB,INPUT_PULLUP);
	pinMode(_pinZ,INPUT_PULLUP);
	pinMode(_pinS,INPUT_PULLUP);
#ifdef TEENSY4_0
#ifdef HALFDUPLEX
	pinMode(_pinRX,INPUT_PULLUP);
#else // HALFDUPLEX
	pinMode(9,    INPUT_PULLUP); //the normal RX pin doesn't work on teensy 4 with full duplex
#endif // HALFDUPLEX
	pinMode(_pinLED,   OUTPUT);
#endif // TEENSY4_0

#ifdef RUMBLE
	pinMode(_pinRumble, OUTPUT);
	pinMode(_pinBrake, OUTPUT);
#endif

	//Teensy 4 has some weird jump in the analog with default pin mode
	pinMode(_pinLa,INPUT_DISABLE);
	pinMode(_pinRa,INPUT_DISABLE);
	pinMode(_pinAx,INPUT_DISABLE);
	pinMode(_pinAy,INPUT_DISABLE);
	pinMode(_pinCx,INPUT_DISABLE);
	pinMode(_pinCy,INPUT_DISABLE);
}

void readButtons(const Pins &pin, Buttons &hardware) {
	hardware.A = !digitalRead(pin.pinA);
	hardware.B = !digitalRead(pin.pinB);
	hardware.X = !digitalRead(pin.pinX);
	hardware.Y = !digitalRead(pin.pinY);
	hardware.L = !digitalRead(pin.pinL);
	hardware.R = !digitalRead(pin.pinR);
	hardware.Z = !digitalRead(pin.pinZ);
	hardware.S = !digitalRead(pin.pinS);
	hardware.Du = !digitalRead(pin.pinDu);
	hardware.Dd = !digitalRead(pin.pinDd);
	hardware.Dl = !digitalRead(pin.pinDl);
	hardware.Dr = !digitalRead(pin.pinDr);
}

void readADCScale(float &ADCScale, float ADCScaleFactor) {
#ifdef USEADCSCALE
	ADCScale = ADCScale*0.999 + ADCScaleFactor/adc->adc1->analogRead(ADC_INTERNAL_SOURCE::VREF_OUT);
#endif
	// otherwise _ADCScale is 1
}

//these are 12 bit but we right shift to get 8 bit
//implement a 3 unit deadzone
int readLa(const Pins &pin, const int initial, const float scale) {
	float temp = (adc->adc0->analogRead(pin.pinLa)) / 16.0;
	if(temp < 3) {
		temp = 0.0f;
	}
	return (uint8_t) min(255, max(0, temp - initial) * scale);
}
int readRa(const Pins &pin, const int initial, const float scale) {
	float temp = (adc->adc0->analogRead(pin.pinRa)) / 16.0;
	if(temp < 3) {
		temp = 0.0f;
	}
	return (uint8_t) min(255, max(0, temp - initial) * scale);
}

//these are native 12-bit
int readAx(const Pins &pin) {
	return adc->adc0->analogRead(pin.pinAx);
}
int readAy(const Pins &pin) {
	return adc->adc0->analogRead(pin.pinAy);
}
int readCx(const Pins &pin) {
	return adc->adc0->analogRead(pin.pinCx);
}
int readCy(const Pins &pin) {
	return adc->adc0->analogRead(pin.pinCy);
}

//output rumble state
void writeRumble(RumbleState state, uint8_t power) {
	if(rumbleState == RUMBLE_ON) {
		analogWrite(_pinBrake, 0);
		analogWrite(_pinRumble, power);
	} else if(rumbleState == RUMBLE_BRAKE) {
		analogWrite(_pinRumble, 0);
		analogWrite(_pinBrake, 255);
	} else {
		analogWrite(_pinRumble, 0);
		analogWrite(_pinBrake, 0);
	}
}

#endif //READHARDWARE_H
