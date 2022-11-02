#ifndef CVIDEO_H
#define CVIDEO_H

#include "structsAndEnums.h"

#ifndef VWIDTH
#define VWIDTH 512
#endif
#ifndef VHEIGHT
#define VHEIGHT 384
#endif

int videoOut(const uint8_t pin_base, Buttons &btn);

uint16_t getImageWidth(const unsigned char image[]);
uint16_t getImageHeight(const unsigned char image[]);
void drawImage(const unsigned char image[],
                const unsigned char index[],
                const uint16_t x,
                const uint16_t y,
                unsigned char bitmap[]);

#endif //CVIDEO_H
