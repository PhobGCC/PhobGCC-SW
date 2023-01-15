#ifndef VERSION_H
#define VERSION_H

#define BUILD_RELEASE
//#define BUILD_DEV

/* This is just an integer. */
#define SW_VERSION 29

/* Uncomment to get a glowing LED on Teensy 4. */
//#define ENABLE_LED

/* Uncomment the appropriate #include line for your hardware by deleting the two slashes at the beginning of the line. */
//#include "teensy/Phob1_0Teensy3_2.h"          // For PhobGCC board 1.0 with Teensy 3.2
//#include "teensy/Phob1_0Teensy3_2DiodeShort.h"// For PhobGCC board 1.0 with Teensy 3.2 and the diode shorted
//#include "teensy/Phob1_1Teensy3_2.h"          // For PhobGCC board 1.1 with Teensy 3.2
//#include "teensy/Phob1_1Teensy3_2DiodeShort.h"// For PhobGCC board 1.1 with Teensy 3.2 and the diode shorted
//#include "teensy/Phob1_1Teensy4_0.h"          // For PhobGCC board 1.1 with Teensy 4.0
//#include "teensy/Phob1_1Teensy4_0DiodeShort.h"// For PhobGCC board 1.1 with Teensy 4.0 and the diode shorted
//#include "teensy/Phob1_2Teensy4_0.h"          // For PhobGCC board 1.2.x with Teensy 4.0
//#include "rp2040/include/PicoProtoboard.h"    // For a protoboard with a Pico on it, used for developing for the RP2040
//#include "rp2040/include/Phob2_0.h"           // For PhobGCC Board 2.0 with RP2040

#ifndef BOARD_H
#error "No hardware selected! Uncomment one of the hardware #include lines above."
#endif

#endif //VERSION_H
