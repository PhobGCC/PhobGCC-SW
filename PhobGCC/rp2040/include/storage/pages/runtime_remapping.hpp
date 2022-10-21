#ifndef PERSISTENCE_PAGES__RUNTIME_REMAPPING_HPP
#define PERSISTENCE_PAGES__RUNTIME_REMAPPING_HPP

#include "pico/stdlib.h"
#include "persistence/page_indexes.hpp"

namespace Persistence {
namespace Pages {

struct RuntimeRemapping {
    static const int index = (int) PageIndexes::RUNTIME_REMAPPING;

    struct __attribute__((packed)) F1GpioToButtonSetRemapping {
        uint8_t configured;

        uint8_t startPin;
        uint8_t rightPin;
        uint8_t downPin;
        uint8_t leftPin;
        uint8_t lPin;
        uint8_t mxPin;
        uint8_t myPin;
        uint8_t cUpPin;
        uint8_t cLeftPin;
        uint8_t aPin;
        uint8_t cDownPin;
        uint8_t cRightPin;
        uint8_t upPin;
        uint8_t msPin;
        uint8_t zPin;
        uint8_t lsPin;
        uint8_t xPin;
        uint8_t yPin;
        uint8_t bPin;
        uint8_t rPin;
    } f1GpioToButtonSetRemapping;
    
};

}
}

#endif