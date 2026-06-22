#ifndef ROBORENESASBACKEND_H
#define ROBORENESASBACKEND_H

#include <Arduino.h>
#include "RoboPwmBackend.h"

#if defined(ROBOSERVO_PLATFORM_RENESAS)

namespace RoboRenesasBackend {
    bool isValidPin(int pin);
    bool attachChannel(uint8_t slot, int pin, int frequency, RoboPwmDomain domain);
    void detachChannel(uint8_t slot);
    void writeDuty(uint8_t slot, uint32_t duty, int frequency, uint8_t resolution);
}

#endif

#endif
