#ifndef BOARD_H
#define BOARD_H

#include <ADC.h>
#include <VREF.h>
#include "debug.h"
#include "settings.h"

//Hardware-specific code for PhobGCC board revision 1.0 with a Teensy 3.2
#define TEENSY3_2

//Hardware specific code for half duplex—using one pin for both TX and RX
#define HALFDUPLEX

//defining which pin is what on the teensy
const int _pinLa = 16;
const int _pinRa = 23;
const int _pinL = 13;
const int _pinR = 3;
const int _pinAx = 15;
const int _pinAy = 14;
const int _pinCx = 22;
const int _pinCy = 21;
const int _pinRX = 9;
const int _pinTX = 10;
const int _pinDr = 7;
const int _pinDu = 18;
const int _pinDl = 17;
const int _pinDd = 8;
const int _pinX = 1;
const int _pinY = 2;
const int _pinA = 4;
const int _pinB = 6;
const int _pinZ = 0;
const int _pinS = 19;
const int _pinLED = 13;
const int _pinInt = 9;

#define USEADCSCALE

void serialSetup() {
    Serial.begin(57600);
    Serial.println("This is the header for board revision 1.0 with a Teensy 3.2.");
}

void ADCSetup(ADC * adc,
              float & ADCScale,
              float & ADCScaleFactor) {
    adc->adc0->setAveraging(1);
    adc->adc0->setResolution(12);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

    adc->adc1->setAveraging(32);
    adc->adc1->setResolution(16);
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);

    VREF::start();

    double refVoltage = 0;
    for (int i = 0; i < 512; i++) {
        int value = adc->adc1->analogRead(ADC_INTERNAL_SOURCE::VREF_OUT);
        double volts = value*3.3/(float)adc->adc1->getMaxValue();
        refVoltage += volts;
    }
    refVoltage = refVoltage/512.0;

    ADCScale = 1.2/refVoltage;

    Serial.print("ADCScale: ");
    Serial.println(ADCScale);

    adc->adc1->setAveraging(1);
    adc->adc1->setResolution(12);
    adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

    ADCScaleFactor = 0.001*1.2*adc->adc1->getMaxValue()/3.3;
    Serial.print("ADCScaleFactor: ");
    Serial.println(ADCScaleFactor);
}

//must include at the end
#include "readHardware.h"
#include "comms.h"
#endif // BOARD_H
