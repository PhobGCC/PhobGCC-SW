#ifndef CVIDEO_VARS_H
#define CVIDEO_VARS_H

#include <stdlib.h>

//ONLY INCLUDE THIS IN CPP FILES, NOT HEADER FILES

/*-------------------------------------------------------------------*/
/*------------------Gray Scale---------------------------------------*/
/*-------------------------------------------------------------------*/
// NTSC in IRE units+40: SYNC = 0; BLANK = 40; BLACK = 47.5; WHITE = 140
const int WHITE = 15;
const int GRAY  = 8;
const int BLACK = 15.0 / 140.0 * 47.5;
const int BLANK = 15.0 / 140.0 * 40.0;
const int SYNC = 0;
//two pixels per byte
const uint8_t BLANK2 = BLANK | BLANK << 4;
const uint8_t BLACK2 = BLACK | BLACK << 4;
const uint8_t GRAY2  = GRAY | GRAY << 4;
const uint8_t WHITE2 = WHITE | WHITE << 4;


#endif //CVIDEO_VARS_H
