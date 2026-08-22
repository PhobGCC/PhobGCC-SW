#pragma once
/*!
 * \file ws2812.h
 */

#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <cstdlib>
#include <string.h>

#include "ws2812.pio.h"

class WS2812
{

public:
    /*!
     * \brief Constructor
     *
     * \param num: Number of pixels in the string
     * \param pin: GPIO pin that controls the NeoPixel string.
     * \param pio: pio selected
     * \param sm: state machine selected
     */
    WS2812(uint16_t num, uint8_t pin, PIO pio, int sm);

    /*!
     * \brief Constructor
     *
     * \param num: Number of pixels in the string
     * \param pin: GPIO pin that controls the NeoPixel string.
     * 
     * This constructor tries to autoselect pio and sm
     */
    WS2812(uint16_t num, uint8_t pin);

    /*!
     * \brief Release memory (as needed):
     */
    virtual ~WS2812();

    /*!
     * \brief Initialize the class instance after calling constructor
     */
    void begin(void);

    /*!
     * \brief Display all the pixels in the buffer
     */
    void show(void);

    /*!
     * \brief Set a NeoPixel to a given color.
     *
     * \param led: set a color for a specific neopixel in the string
     * \param red: red value (0-255)
     * \param green: green value(0-255)
     * \param blue: blue value (0-255)
     */
    void setPixelColor(uint16_t led, uint8_t red, uint8_t green, uint8_t blue);

    /*!
     * \brief set a pixel color fomr 'packed' 32-bit RGB value
     *
     * \param led: specific neopixel in the string
     * \param color packed 32-bit RGB value to set the pixel
     */
    void setPixelColor(uint16_t led, uint32_t color);

    /*!
     * \brief Clear the entire string
     */
    void clear();

    /*! 
     * \brief Fill all the pixels with same value
     *
     * \param red: red value (0-255)
     * \param green: green value(0-255)
     * \param blue: blue value (0-255)
     */
    void fillPixelColor(uint8_t red, uint8_t green, uint8_t blue);

    /*!
     * \brief Change strand length
     *
     * \param numLEDs Length to change to
     */
    void updateLength(uint16_t num);
    
    /*!
     * \brief Returns the number of pixels currently connected
     *
     * \return Returns the number of connected pixels
     */
    uint16_t numPixels(void);
    
    /*!
     * \brief Query color from previously-set pixel
     *
     * \param led Pixel to query
     * \return Returns packed 32-bit RGB value
     */
    uint32_t getPixelColor(uint16_t led);

private:
    uint16_t numLEDs; // number of pixels

    // a buffer that holds the color for each pixel
    uint8_t *pixels;

    uint8_t pixelGpio;

    // pio - 0 or 1
    PIO pixelPio;

    // pio state machine to use
    int pixelSm;

    // calculated program offset in memory
    uint pixelOffset;

    void alloc(uint16_t n);

};
