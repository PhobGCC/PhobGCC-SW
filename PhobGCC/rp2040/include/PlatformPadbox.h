#ifndef BOARD_H
#define BOARD_H

#include "ws2812.h"
#define CLEANADC

//This is for the Arkodd Platform Padbox GS

//defining which pin is what
//GPIO
const int _pinA  = 15;
const int _pinB  = 6;  //K1, bottom row 1
const int _pinL  = 22; //L3, left (only) trigger
const int _pinR  = 10; //P1, top row 1
const int _pinX  = 7;  //K2, bottom row 2
const int _pinY  = 11; //P2, top row 2
const int _pinZ  = 8; //K3, bottom row 3
const int _pinS  = 17; //working
const int _pinRumble = -1;
const int _pinBrake  = -1;
const int _pinTX  = 23; //working
//20 is actually the third menu button
//9 is dpad left
//7 was dpad left, now it does nothing?

//Analog trigger values
#define B0XXRIGHT
const int _pinLS = 12; //P3, top row 3
const int _pinMS = 13; //P4, top row 4
const int _pinUP = 9;  //K4, bottom row 4 (unused by default)

//Permanent d-pad
#define DPAD
const int _pinDr = 4;
const int _pinDu = 2;
const int _pinDl = 5;
const int _pinDd = 3;

//Digital C-button inputs
#define CBUTTONS
const int _pinCr = 19;
const int _pinCu = 27;
const int _pinCl = 26;
const int _pinCd = 18;

#define ASTICK_RP_ADC
const int _pinAx = 29;
const int _pinAxADC = 3;
const int _pinAy = 28;
const int _pinAyADC = 2;
const int _pinCx = -1;
const int _pinCxADC = -1;
const int _pinCy = -1;
const int _pinCyADC = -1;
const int _pinRX = -1;

#define NEOPIXEL_CHAIN
#include <memory>
#include "ws2812.h"
const int _pinLED = 14;
const int _ledCount = 13;
void writeLED(WS2812 * neopixel, const int onlyPin = -1) {
    if(onlyPin >=  0) {neopixel->clear();}
    if(onlyPin < 0 || onlyPin ==  0) {neopixel->setPixelColor( 4, 90,  0,  0);}//left to right, top to bottom
    if(onlyPin < 0 || onlyPin ==  1) {neopixel->setPixelColor( 5, 60, 30,  0);}
    if(onlyPin < 0 || onlyPin ==  2) {neopixel->setPixelColor( 6, 30, 60,  0);}
    if(onlyPin < 0 || onlyPin ==  3) {neopixel->setPixelColor( 7,  0, 90,  0);}
    if(onlyPin < 0 || onlyPin ==  4) {neopixel->setPixelColor( 3,  0, 60, 30);}
    if(onlyPin < 0 || onlyPin ==  5) {neopixel->setPixelColor( 2,  0, 30, 60);}
    if(onlyPin < 0 || onlyPin ==  6) {neopixel->setPixelColor( 1,  0,  0, 90);}
    if(onlyPin < 0 || onlyPin ==  7) {neopixel->setPixelColor( 0, 30,  0, 60);}
    if(onlyPin < 0 || onlyPin ==  8) {neopixel->setPixelColor( 8,  0, 90,  0);}//a = green
    if(onlyPin < 0 || onlyPin ==  9) {neopixel->setPixelColor( 9, 45, 45,  0);}//c buttons = yellow
    if(onlyPin < 0 || onlyPin == 10) {neopixel->setPixelColor(10, 45, 45,  0);}
    if(onlyPin < 0 || onlyPin == 11) {neopixel->setPixelColor(11, 45, 45,  0);}
    if(onlyPin < 0 || onlyPin == 12) {neopixel->setPixelColor(12, 45, 45,  0);}
    neopixel->show();
}

#include "debug.h"
#include "readHardware.h"

#endif //BOARD_H
