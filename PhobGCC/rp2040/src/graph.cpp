#include <algorithm>
#include "cvideo.h"

using std::min;
using std::max;

//Graphing routines go here

//Draw x/y parametric plot
//x/y gets clamped to -127 to +127
void graphStickmap(unsigned char bitmap[],
		const uint16_t x0,
		const uint16_t y0,
		const int x[],
		const int y[],
		const uint16_t length,
		const uint8_t color,
		const LineOrPoint graphType) {
	if((VWIDTH <= x0 + 256) || (VHEIGHT <= y0 + 256)) {
		return;
	}
	if(graphType == POINTGRAPH) {
		for(int point = 0; point < length; point++) {
			const int tempX = min(127, max(-127,  x[point])) + 127 + x0;
			const int tempY = min(127, max(-127, -y[point])) + 127 + y0;

			const unsigned int rowOffset = tempY*VWIDTH/2;
			if(tempX % 2) { //odd
				bitmap[rowOffset + tempX/2] = (bitmap[rowOffset + tempX/2]&0xF0) | (color);
			} else { //even: shift left by 4
				bitmap[rowOffset + tempX/2] = (bitmap[rowOffset + tempX/2]&0x0F) | (color << 4);
			}
		}
	} else {
		if(length <= 1) {
			return;
		}
		int oldX = min(127, max(-127,  x[0]+127)) + x0;
		int oldY = min(127, max(-127, -y[0]+127)) + y0;
		for(int point = 1; point < length; point++) {
			const int tempX = min(127, max(-127,  x[point])) + 127 + x0;
			const int tempY = min(127, max(-127, -y[point])) + 127 + y0;

			drawLine(bitmap, oldX, oldY, tempX, tempY, color);

			oldX = tempX;
			oldY = tempY;
		}
	}
}
