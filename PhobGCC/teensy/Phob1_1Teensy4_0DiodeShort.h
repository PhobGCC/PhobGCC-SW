#ifndef BOARD_H
#define BOARD_H

#include <ADC.h>
#include <VREF.h>
#include "debug.h"
#include "settings.h"

//Hardware specific code for PhobGCC board revision 1.1 with a Teensy 4.0
#define TEENSY4_0

//Hardware specific code for half duplex—using one pin for both TX and RX
#define HALFDUPLEX

//defining which pin is what on the teensy
const int _pinLa = 16;
const int _pinRa = 23;
const int _pinL = 12;
const int _pinR = 3;
const int _pinAx = 15;
const int _pinAy = 14;
const int _pinCx = 22;
const int _pinCy = 21;
const int _pinRX = 7;
const int _pinTX = 8;
const int _pinDr = 6;
const int _pinDu = 18;
const int _pinDl = 17;
const int _pinDd = 11;
const int _pinX = 1;
const int _pinY = 2;
const int _pinA = 4;
const int _pinB = 20;
const int _pinZ = 0;
const int _pinS = 19;
const int _pinLED = 13;
const int _pinInt = 7;

//don't #define USEADCSCALE

void serialSetup() {
    Serial.begin(115200);
    Serial.println("This is the header for board revision 1.1 with a Teensy 4.0.");
}

void ADCSetup(ADC * adc,
              float &,/*ADCScale not used*/
              float & /*ADCScaleFactor not used*/) {
    adc->adc0->setAveraging(1);
    adc->adc0->setResolution(12);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

}

//must include at the end
#include "readHardware.h"
#include "comms.h"
#endif // BOARD_H
