/*!
 * \file ws2812.cpp
 *
 * \mainpage WS2812 Library
 *
 * \section intro_sec Introduction
 *
 * Example to control WS2812-based RGB LED Modules in a strand or strip with RP2040 SDK only
 *
 * \section author Author
 *
 * Written by PBeal
 * 
 * \section license License
 * GNU LESSER GENERAL PUBLIC LICENSE, V3.0
 */
#include "ws2812.h"

WS2812::WS2812(uint16_t num, uint8_t pin, PIO pio, int sm) {
    alloc(num);

    pixelSm = sm;
    pixelPio = pio;
    pixelGpio = pin;
}

WS2812::WS2812(uint16_t num, uint8_t pin) {
    alloc(num);
    
    PIO pio = pio0;
    int sm;
    // Find a free SM on one of the PIO's
    sm = pio_claim_unused_sm(pio, false); // don't panic
    // Try pio1 if SM not found
    if (sm < 0) {
        pio = pio1;
        sm = pio_claim_unused_sm(pio, true); // panic if no SM is free
    }
    
    pixelSm = sm;
    pixelPio = pio;
    pixelGpio = pin;
}

void WS2812::begin() {
    uint offset = pio_add_program(pixelPio, &ws2812_program);
    ws2812_program_init(pixelPio, pixelSm, offset, pixelGpio, 800000, false);
}

// Allocate 3 bytes per pixel, init to RGB 'off' state:
void WS2812::alloc(uint16_t num)
{
    numLEDs = ((pixels = (uint8_t *)calloc(num, 3)) != NULL) ? num : 0;
}

// Release memory (as needed):
WS2812::~WS2812(void)
{
    if (pixels)
        free(pixels);
}

// Set pixel color from separate 8-bit R, G, B components:
void WS2812::setPixelColor(uint16_t led, uint8_t red, uint8_t green, uint8_t blue)
{
    if (led < numLEDs)
    {
        uint8_t *p = &pixels[led * 3];
        *p++ = red;
        *p++ = green;
        *p++ = blue;
    }
}

// Set pixel color from 'packed' 32-bit RGB value:
void WS2812::setPixelColor(uint16_t led, uint32_t color)
{
    if (led < numLEDs)
    {
        uint8_t *p = &pixels[led * 3];
        *p++ = color >> 16; // Red
        *p++ = color >> 8;  // Green
        *p++ = color;       // Blue
    }
}

// Clear all pixels
void WS2812::clear()
{
    if (pixels != NULL)
    {
        memset(pixels, 0, numLEDs * 3);
    }
}

// Update the WS2812 pixels
void WS2812::show(void) {
    for (uint16_t i = 0; i < numLEDs; i++) 
    {
        uint8_t redPtr = this->pixels[i*3];
        uint8_t greenPtr = this->pixels[(i*3)+1];
        uint8_t bluePtr = this->pixels[(i*3)+2];
        uint32_t colorData = ((uint32_t)(redPtr) << 8) | ((uint32_t)(greenPtr) << 16) | (uint32_t)(bluePtr);

        pio_sm_put_blocking(pixelPio, pixelSm, colorData << 8u);
    }
}

void WS2812::fillPixelColor(uint8_t red, uint8_t green, uint8_t blue)
{
    // fill all the neopixels in the buffer with the specified rgb values.
    for (uint16_t i = 0; i < numLEDs; i++)
    {    
        uint8_t *p = &pixels[i * 3];
        *p++ = red;
        *p++ = green;
        *p++ = blue;
    }
}

void WS2812::updateLength(uint16_t num)
{
    if (pixels != NULL)
        free(pixels); // Free existing data (if any)
    // Allocate new data -- note: ALL PIXELS ARE CLEARED
    numLEDs = ((pixels = (uint8_t *)calloc(num, 3)) != NULL) ? num : 0;
}

uint16_t WS2812::numPixels(void) { return numLEDs; }

uint32_t WS2812::getPixelColor(uint16_t led)
{
    if (led < numLEDs)
    {
        uint16_t ofs = led * 3;
        // To keep the show() loop as simple & fast as possible, the
        // internal color representation is native to different pixel
        // types.  For compatibility with existing code, 'packed' RGB
        // values passed in or out are always 0xRRGGBB order.
        return ((uint32_t)pixels[ofs] << 16) | ((uint16_t)pixels[ofs + 1] << 8) | ((uint16_t)pixels[ofs + 2]);
    }

    return 0; // Pixel # is out of bounds
}
