#include "cvideo.h"

//Misc graphics drawing routines go here.

//Draw a line from the first point to the second point including both.
//This implements Bresenham's line algorithm
void drawLineLow(unsigned char bitmap[],
                 const uint16_t x0,
                 const uint16_t y0,
                 const uint16_t x1,
                 const uint16_t y1,
				 const uint8_t color) {
	const int dx = x1 - x0;
	const int dy = y1 - y0;
	const int yi = 1;
	if(dy < 0) {
		yi = -1;
		dy = -dy;
	}

	int D = (2*dy) - dx;
	int y = y0;

	for(int x = x0; x <= x1; x++) {
		uint32_t rowOffset = y*VWIDTH/2;
		if(x % 2) { //odd; shift left by 4
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0x0F) | (color << 4);
		} else { //even
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0xF0) | (color);
		}
		if(D > 0) {
			y = y + yi;
			D = D + (2 * (dy - dx));
		} else {
			D = D + 2*dy;
		}
	}
}
void drawLineHigh(unsigned char bitmap[],
                  const uint16_t x0,
                  const uint16_t y0,
                  const uint16_t x1,
                  const uint16_t y1,
				  const uint8_t color) {
	const int dx = x1 - x0;
	const int dy = y1 - y0;
	const int xi = 1;
	if(dx < 0) {
		xi = -1;
		dx = -dx;
	}

	int D = (2*dx) - dy;
	int x = x0;

	for(int y = y0; y <= y1; y++) {
		uint32_t rowOffset = y*VWIDTH/2;
		if(x % 2) { //odd; shift left by 4
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0x0F) | (color << 4);
		} else { //even
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0xF0) | (color);
		}
		if(D > 0) {
			x = x + xi;
			D = D + (2 * (dx - dy));
		} else {
			D = D + 2*dx;
		}
	}
}
void drawLine(unsigned char bitmap[],
              const uint16_t x0,
              const uint16_t y0,
              const uint16_t x1,
              const uint16_t y1,
			  const uint8_t color) {
	if(abs(y1-y0) < abs(x1-x0)) {
		if(x0 > x1) {
			drawLineLow(bitmap, x1, y1, x0, y0, color);
		} else {
			drawLineLow(bitmap, x0, y0, x1, y1, color);
		}
	} else {
		if(y0 > y1) {
			drawLineHigh(bitmap, x1, y1, x0, y0, color);
		} else {
			drawLineHigh(bitmap, x0, y0, x1, y1, color);
		}
	}
}

