#ifndef __JOYBUS_HPP
#define __JOYBUS_HPP

#include "pico/stdlib.h"
#include "gcReport.hpp"

#include <functional>

/**
 * @short Enters the Joybus communication mode
 * 
 * @param dataPin GPIO number of the console data line pin
 * @param func Function to be called to obtain the GCReport to be sent to the console
 */
void enterMode(const int dataPin, const int rumblePin, const int brakePin, int &rumblePower, std::function<GCReport()> func);

#endif
