#include <charconv>
#include <cmath>
#include <cstring>
#include <string>
#include "pico/platform.h"
#include "cvideo.h"
#include "images/font.h"

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
	int dy = y1 - y0;
	int yi = 1;
	if(dy < 0) {
		yi = -1;
		dy = -dy;
	}

	int D = (2*dy) - dx;
	int y = y0;

	for(int x = x0; x <= x1; x++) {
		uint32_t rowOffset = y*VWIDTH/2;
		if(x % 2) { //odd
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0xF0) | (color);
		} else { //even: shift left by 4
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0x0F) | (color << 4);
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
	int dx = x1 - x0;
	const int dy = y1 - y0;
	int xi = 1;
	if(dx < 0) {
		xi = -1;
		dx = -dx;
	}

	int D = (2*dx) - dy;
	int x = x0;

	for(int y = y0; y <= y1; y++) {
		uint32_t rowOffset = y*VWIDTH/2;
		if(x % 2) { //odd
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0xF0) | (color);
		} else { //even: shift left by 4
			bitmap[rowOffset + x/2] = (bitmap[rowOffset + x/2]&0x0F) | (color << 4);
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
	if(x0 >= VWIDTH || x1 >= VWIDTH || y0 >= VHEIGHT || y1 >= VHEIGHT) {
		return;
	}
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

//Draws 8x15 character in the specified location according to the ascii codepoints
void drawChar(unsigned char bitmap[],
              const uint16_t x,
			  const uint16_t y,
			  const uint8_t color,
			  const char character) {
	if(character < 0x20 || character > 0x7e) { //lower than space, larger than tilde
		return;
	}
	if(x + 8-1 >= VWIDTH || y + 15-1 >= VHEIGHT) { //out of bounds
	//if(x + 16-1 >= VWIDTH || y + 30-1 >= VHEIGHT) { //out of bounds
		return;
	}
	for(int row = 0; row < 15; row++) {
		uint32_t rowOffset = (row+y)*VWIDTH/2;
		for(int col = 0; col < 8; col++) {
			if((ascii::font[(character-0x20)*15+row] << col) & 0b10000000) {
			//if((ascii::font[(character-0x20)*15+(row/2)] << (col/2)) & 0b10000000) {
			//if((ascii::font[row/2] << (col/2)) & 0b10000000) {
				uint16_t colOffset = col+x;
				if(colOffset % 2) { //odd
					bitmap[rowOffset + colOffset/2] = (bitmap[rowOffset + colOffset/2]&0xF0) | (color);
				} else { //even: shift left by 4
					bitmap[rowOffset + colOffset/2] = (bitmap[rowOffset + colOffset/2]&0x0F) | (color << 4);
				}
			}
		}
	}
}

void drawString(unsigned char bitmap[],
                const uint16_t x0,
			    const uint16_t y0,
			    const uint8_t color,
			    const char string[],
                const uint8_t charLimit) {
	uint16_t i = 0;
	const char nullChar[] = "";
	while(string[i] != nullChar[0] && i < charLimit) {
		int x = x0 + 10*i;
		drawChar(bitmap, x, y0, color, string[i]);
		i++;
	}
}

//Draws 8x15 character in the specified location according to the ascii codepoints
void drawChar2x(unsigned char bitmap[],
                const uint16_t x,
			    const uint16_t y,
			    const uint8_t color,
			    const char character) {
	if(character < 0x20 || character > 0x7e) { //lower than space, larger than tilde
		return;
	}
	if(x + 16-1 >= VWIDTH || y + 30-1 >= VHEIGHT) { //out of bounds
		return;
	}
	for(int row = 0; row < 15; row++) {
		uint32_t rowOffset1 = (row*2  +y)*VWIDTH/2;
		uint32_t rowOffset2 = (row*2+1+y)*VWIDTH/2;
		for(int col = 0; col < 8; col++) {
			if((ascii::font[(character-0x20)*15+row] << col) & 0b10000000) {
			//if((ascii::font[(character-0x20)*15+(row/2)] << (col/2)) & 0b10000000) {
			//if((ascii::font[row/2] << (col/2)) & 0b10000000) {
				uint16_t colOffset = col*2+x;
				if(colOffset % 2) {
					//odd
					bitmap[rowOffset1 + colOffset/2] = (bitmap[rowOffset1 + colOffset/2]&0xF0) | (color);
					bitmap[rowOffset2 + colOffset/2] = (bitmap[rowOffset2 + colOffset/2]&0xF0) | (color);
					//even: shift left by 4
					bitmap[rowOffset1 + colOffset/2 + 1] = (bitmap[rowOffset1 + colOffset/2 + 1]&0x0F) | (color << 4);
					bitmap[rowOffset2 + colOffset/2 + 1] = (bitmap[rowOffset2 + colOffset/2 + 1]&0x0F) | (color << 4);
				} else {
					//even: shift left by 4
					bitmap[rowOffset1 + colOffset/2] = (bitmap[rowOffset1 + colOffset/2]&0x0F) | (color << 4);
					bitmap[rowOffset2 + colOffset/2] = (bitmap[rowOffset2 + colOffset/2]&0x0F) | (color << 4);
					//odd
					bitmap[rowOffset1 + colOffset/2] = (bitmap[rowOffset1 + colOffset/2]&0xF0) | (color);
					bitmap[rowOffset2 + colOffset/2] = (bitmap[rowOffset2 + colOffset/2]&0xF0) | (color);
				}
			}
		}
	}
}

void drawString2x(unsigned char bitmap[],
                  const uint16_t x0,
			      const uint16_t y0,
			      const uint8_t color,
			      const char string[],
                  const uint8_t charLimit) {
	uint16_t i = 0;
	const char nullChar[] = "";
	while(string[i] != nullChar[0] && i < charLimit) {
		int x = x0 + 20*i;
		drawChar2x(bitmap, x, y0, color, string[i]);
		i++;
	}
}

void drawFloat(unsigned char bitmap[],
               const uint16_t x0,
               const uint16_t y0,
               const uint8_t color,
               const uint8_t largestPower,
			   const uint8_t totalChars,
               const float number) {
	int offset = (number >= 0) ? 1 : 0;
	int decimal = 1;
	for(int i = 0; i < largestPower; i++) {
		decimal *= 10;
		if(abs(number) < decimal) {
			offset += 1;
		}
	}
	int charLimit = 99;
	if(totalChars >= largestPower+2) {
		charLimit = totalChars-offset;
	}
	offset *= 10;

	std::string numberString = std::to_string(number);
	const char* numberChar = numberString.c_str();

	drawString(bitmap, x0 + offset, y0, color, numberChar, charLimit);
}
void drawFloat2x(unsigned char bitmap[],
                 const uint16_t x0,
                 const uint16_t y0,
                 const uint8_t color,
                 const uint8_t largestPower,
				 const uint8_t totalChars,
                 const float number) {
	int offset = (number >= 0) ? 1 : 0;
	int decimal = 1;
	for(int i = 0; i < largestPower; i++) {
		decimal *= 10;
		if(abs(number) < decimal) {
			offset += 1;
		}
	}
	int charLimit = 99;
	if(totalChars >= largestPower+2) {
		charLimit = totalChars-offset;
	}
	offset *= 20;

	std::string numberString = std::to_string(number);
	const char* numberChar = numberString.c_str();

	drawString2x(bitmap, x0 + offset, y0, color, numberChar);
}
void drawInt(unsigned char bitmap[],
             const uint16_t x0,
             const uint16_t y0,
             const uint8_t color,
             const uint8_t largestPower,
             const int number) {
	int offset = (number >= 0) ? 1 : 0;
	int decimal = 1;
	for(int i = 0; i < largestPower; i++) {
		decimal *= 10;
		if(abs(number) < decimal) {
			offset += 1;
		}
	}
	offset *= 10;

	std::string numberString = std::to_string(number);
	const char* numberChar = numberString.c_str();

	drawString(bitmap, x0 + offset, y0, color, numberChar);
}
void drawInt2x(unsigned char bitmap[],
               const uint16_t x0,
               const uint16_t y0,
               const uint8_t color,
               const uint8_t largestPower,
               const int number) {
	int offset = (number >= 0) ? 1 : 0;
	int decimal = 1;
	for(int i = 0; i < largestPower; i++) {
		decimal *= 10;
		if(abs(number) < decimal) {
			offset += 1;
		}
	}
	offset *= 20;

	std::string numberString = std::to_string(number);
	const char* numberChar = numberString.c_str();

	drawString2x(bitmap, x0 + offset, y0, color, numberChar);
}
