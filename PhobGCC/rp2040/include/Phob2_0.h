#ifndef BOARD_H
#define BOARD_H

#ifndef RUMBLE
#define RUMBLE
#endif //RUMBLE

#ifndef CLEANADC
#define CLEANADC
#endif //CLEANADC

//defining which pin is what
//GPIO
const int _pinA  = 17;
const int _pinB  = 16;
const int _pinDr = 11;
const int _pinDu = 9;
const int _pinDl = 8;
const int _pinDd = 10;
const int _pinL  = 22;
const int _pinR  = 21;
const int _pinX  = 18;
const int _pinY  = 19;
const int _pinZ  = 20;
const int _pinS  = 5;
const int _pinRumble = 25;
const int _pinBrake  = 29;
const int _pinTX  = 28;
//GPIO SPI for ADCs
const int _pinSPIclk = 6;
const int _pinSPItx  = 7;
const int _pinSPIrx  = 4;
const int _pinAcs    = 24;
const int _pinCcs    = 23;
//a little resistor ladder DAC
const int _pinDac0 = 0;
const int _pinDac1 = 1;
const int _pinDac2 = 2;
const int _pinDac3 = 3;
//this is only for the pico itself, not necessarily the phob
//two of the built-in ADCs:
const int _pinRa = 26; //GPIO number
const int _pinRadc = 0; //ADC number
const int _pinLa = 27; //GPIO number
const int _pinLadc = 1; //ADC number
//and four more spare connections
const int _pinSpare0 = 12;
const int _pinSpare1 = 13;
const int _pinSpare2 = 14;
const int _pinLED    = 15;

const int _pinAx = -1;
const int _pinAy = -1;
const int _pinCx = -1;
const int _pinCy = -1;
const int _pinRX = -1;

#include "debug.h"
#include "readHardware.h"

#endif //BOARD_H
