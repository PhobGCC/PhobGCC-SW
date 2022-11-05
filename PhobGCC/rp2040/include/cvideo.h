#ifndef CVIDEO_H
#define CVIDEO_H

#include "structsAndEnums.h"

#ifndef VWIDTH
#define VWIDTH 512
#endif
#ifndef VHEIGHT
#define VHEIGHT 384
#endif
#ifndef BUFFERLEN
#define BUFFERLEN 98304//512*384/2
#endif

int videoOut(const uint8_t pin_base, Buttons &btn);

uint16_t getImageWidth(const unsigned char image[]);
uint16_t getImageHeight(const unsigned char image[]);
void drawImage(unsigned char bitmap[],
               const unsigned char image[],
               const unsigned char index[],
               const uint16_t x,
               const uint16_t y);

void drawLine(unsigned char bitmap[],
              const uint16_t xStart,
              const uint16_t yStart,
              const uint16_t xEnd,
              const uint16_t yEnd,
              const uint8_t color);

#endif //CVIDEO_H
