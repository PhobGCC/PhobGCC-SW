#ifndef __GCREPORT_HPP
#define __GCREPORT_HPP

#include "pico/stdlib.h"

struct __attribute__((packed)) GCReport {
    uint8_t a : 1; uint8_t b : 1; uint8_t x:1; uint8_t y : 1; uint8_t start : 1; uint8_t pad0 : 3;
    uint8_t dLeft : 1; uint8_t dRight : 1; uint8_t dDown : 1; uint8_t dUp : 1; uint8_t z : 1; uint8_t r : 1; uint8_t l : 1; uint8_t pad1 : 1;
    uint8_t xStick;
    uint8_t yStick;
    uint8_t cxStick;
    uint8_t cyStick;
    uint8_t analogL;
    uint8_t analogR;
};

const GCReport defaultGcReport = {
    .a=0, .b=0, .x=0, .y=0, .start=0, .pad0=0,
    .dLeft=0, .dRight=0, .dDown=0, .dUp=0, .z=0, .r=0, .l=0, .pad1=1,
    .xStick=127,
    .yStick=127,
    .cxStick=127,
    .cyStick=127,
    .analogL=0,
    .analogR=0
};

#endif
