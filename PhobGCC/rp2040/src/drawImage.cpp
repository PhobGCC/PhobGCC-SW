#include "cvideo.h"

/* Decode runlength encoded images of the following format:
 * unsigned char image[]:
 *   First two bytes are width
 *   Third and fourth bytes are height
 *   Each run gets a byte.
 *     The first five bits are run length-1; take this value and add 1 to get the run length.
 *     The last three bits are an index.
 * unsigned char index[8]:
 *   There must be eight values.
 *   A value of 4 or less indicates transparency: nothing is written
 *   Other values must be from 5 through 15 inclusive. 5 is black, 15 is white.
 *   These do not have to be in order.
 */


uint16_t getImageWidth(const unsigned char image[]){
	return (image[0]<<8) | image[1];
}
uint16_t getImageHeight(const unsigned char image[]){
	return (image[2]<<8) | image[3];
}

void drawImage(unsigned char bitmap[],
               const unsigned char image[],
               const unsigned char index[8],
               const uint16_t x,
               const uint16_t y) {
	uint32_t imgWidth = getImageWidth(image);
	uint32_t imgHeight = getImageHeight(image);
	uint32_t pixelCount = imgWidth * imgHeight;

	uint32_t xEnd = x + imgWidth;
	uint32_t yEnd = y + imgHeight;
	if(xEnd > VWIDTH || yEnd > VHEIGHT) {
		//don't draw images that will go out of bounds
		return;
	}

	//starting byte for decoding the run length encoding
	uint32_t byte = 4;
	uint8_t runIndex = 0;
	uint8_t runLength = (image[byte] >> 3) + 1;
	uint8_t color = index[image[byte] & 0b111];
	for(uint16_t row = y; row < yEnd; row++) {
		uint32_t rowOffset = row*VWIDTH/2;
		for(uint16_t col = x; col < xEnd; col++) {
			if(color >= 5) {
				if(col % 2) { //odd
					bitmap[rowOffset + col/2] = (bitmap[rowOffset + col/2]&0xF0) | (color);
				} else { //even: shift left by 4
					bitmap[rowOffset + col/2] = (bitmap[rowOffset + col/2]&0x0F) | (color << 4);
				}
			}
			runIndex++;
			if(runIndex >= runLength) {
				runIndex = 0;
				byte++;
				runLength = (image[byte] >> 3) + 1;
				color = index[image[byte] & 0b111];
			}
		}
	}
}
