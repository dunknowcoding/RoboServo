/**
 * @file RoboAvrBackend.h
 * @brief Timer1 ISR servo backend for AVR (internal)
 */

#ifndef ROBOAVRBACKEND_H
#define ROBOAVRBACKEND_H

#include <Arduino.h>
#include "RoboPwmBackend.h"

#if defined(ROBOSERVO_PLATFORM_AVR)

namespace RoboAvrBackend {

    bool isValidPin(int pin);
    bool attachChannel(uint8_t slot, int pin, int frequency, RoboPwmDomain domain);
    void detachChannel(uint8_t slot);
    void writeDuty(uint8_t slot, uint32_t duty, int frequency, uint8_t resolution);

}

#endif // ROBOSERVO_PLATFORM_AVR

#endif // ROBOAVRBACKEND_H
