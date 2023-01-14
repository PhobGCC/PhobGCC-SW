#ifndef BOARD_H
#define BOARD_H

#ifndef CLEANADC
#define CLEANADC
#endif //CLEANADC

//defining which pin is what
//GPIO
//TODO: these are all temporary
const int _pinA =  1;
const int _pinB =  2;
const int _pinDr = 9;
const int _pinDu = 21;
const int _pinDl = 22;
const int _pinDd = 28;
const int _pinL =  3;
const int _pinR =  4;
const int _pinX =  5;
const int _pinY =  6;
const int _pinZ =  7;
const int _pinS =  8;
const int _pinRumble = 24;
const int _pinBrake = 14;
const int _pinTX  = 15;
//GPIO SPI for ADCs
const int _pinSPIrx = 16;
const int _pinAcs = 17;
const int _pinSPIclk = 18;
const int _pinSPItx = 19;
const int _pinCcs = 20;
//a little resistor ladder DAC
const int _pinDac0 = 10;
const int _pinDac1 = 11;
const int _pinDac2 = 12;
const int _pinDac3 = 13;
//this is only for the pico itself, not necessarily the phob
const int _pinLED = 25;
//two of the built-in ADCs:
const int _pinRa = 26; //GPIO number
const int _pinRadc = 0; //ADC number
const int _pinLa = 27; //GPIO number
const int _pinLadc = 1; //ADC number
//and two more
const int _pinSpare0 = 0;
const int _pinSpare1 = 23;
const int _pinSpare2 = 29;

const int _pinAx = -1;
const int _pinAy = -1;
const int _pinCx = -1;
const int _pinCy = -1;
const int _pinRX = -1;

#include "debug.h"
#include "readHardware.h"

#endif //BOARD_H
