#include "RoboPlatform.h"

#if defined(ROBOSERVO_PLATFORM_RENESAS)

#include "RoboServo.h"
#include "RoboRenesasBackend.h"
#include "pwm.h"

static PwmOut* _pwmOut[ROBOSERVO_MAX_SERVOS] = {};
static int _frameFrequency = ROBOSERVO_DEFAULT_FREQUENCY;

static uint32_t dutyToPulseUs(uint32_t duty, int frequency, uint8_t resolution) {
    const uint32_t maxDuty = (1UL << resolution) - 1UL;
    const uint32_t periodUs = (frequency > 0) ? (1000000UL / (uint32_t)frequency) : 20000UL;
    if (maxDuty == 0) return 1500UL;
    return (duty * periodUs) / maxDuty;
}

namespace RoboRenesasBackend {

bool isValidPin(int pin) {
    #ifdef digitalPinHasPWM
    return digitalPinHasPWM(pin);
    #else
    return pin >= 0 && pin < PINS_COUNT;
    #endif
}

bool attachChannel(uint8_t slot, int pin, int frequency, RoboPwmDomain domain) {
    if (domain != ROBOPWM_DOMAIN_SERVO) return false;
    if (slot >= ROBOSERVO_MAX_SERVOS || !isValidPin(pin)) return false;
    if (frequency < ROBOSERVO_MIN_FREQUENCY || frequency > ROBOSERVO_MAX_FREQUENCY) return false;

    _frameFrequency = frequency;
    const uint32_t periodUs = 1000000UL / (uint32_t)frequency;

    if (_pwmOut[slot] != nullptr) {
        delete _pwmOut[slot];
        _pwmOut[slot] = nullptr;
    }

    PwmOut* pwm = new PwmOut(pin);
    if (pwm == nullptr || !pwm->begin(periodUs, 1500UL, false)) {
        delete pwm;
        return false;
    }

    _pwmOut[slot] = pwm;
    return true;
}

void detachChannel(uint8_t slot) {
    if (slot >= ROBOSERVO_MAX_SERVOS) return;
    if (_pwmOut[slot] != nullptr) {
        _pwmOut[slot]->end();
        delete _pwmOut[slot];
        _pwmOut[slot] = nullptr;
    }
}

void writeDuty(uint8_t slot, uint32_t duty, int frequency, uint8_t resolution) {
    if (slot >= ROBOSERVO_MAX_SERVOS || _pwmOut[slot] == nullptr) return;

    if (frequency > 0) {
        _frameFrequency = frequency;
    } else {
        frequency = _frameFrequency;
    }

    const uint32_t periodUs = 1000000UL / (uint32_t)frequency;
    const uint32_t pulseUs = dutyToPulseUs(duty, frequency, resolution);
    _pwmOut[slot]->begin(periodUs, pulseUs, false);
}

}

#endif
