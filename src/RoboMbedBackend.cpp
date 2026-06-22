#include "RoboPlatform.h"

#if defined(ROBOSERVO_PLATFORM_MBED)

#include <mbed.h>
#include "RoboServo.h"
#include "RoboMbedBackend.h"

class MbedServoPulse {
public:
    explicit MbedServoPulse(PinName pin)
        : _pin(pin), _out(new mbed::DigitalOut(pin)), _pulseUs(1500), _running(false) {}

    ~MbedServoPulse() {
        stop();
        delete _out;
    }

    void start(uint32_t pulseUs) {
        _pulseUs = pulseUs;
        if (!_running) {
            _ticker.attach(mbed::callback(this, &MbedServoPulse::onFrame), 0.02f);
            _running = true;
        }
    }

    void stop() {
        _ticker.detach();
        _timeout.detach();
        if (_out != nullptr) {
            _out->write(0);
        }
        _running = false;
    }

private:
    void onFrame() {
        _out->write(1);
        _timeout.attach(mbed::callback(this, &MbedServoPulse::endPulse), _pulseUs / 1000000.0f);
    }

    void endPulse() {
        _out->write(0);
    }

    PinName _pin;
    mbed::DigitalOut* _out;
    mbed::Ticker _ticker;
    mbed::Timeout _timeout;
    uint32_t _pulseUs;
    bool _running;
};

static MbedServoPulse* _channels[ROBOSERVO_MAX_SERVOS] = {};
static int _frameFrequency = ROBOSERVO_DEFAULT_FREQUENCY;

static uint32_t dutyToPulseUs(uint32_t duty, int frequency, uint8_t resolution) {
    const uint32_t maxDuty = (1UL << resolution) - 1UL;
    const uint32_t periodUs = (frequency > 0) ? (1000000UL / (uint32_t)frequency) : 20000UL;
    if (maxDuty == 0) return 1500UL;
    return (duty * periodUs) / maxDuty;
}

namespace RoboMbedBackend {

bool isValidPin(int pin) {
    return pin >= 0 && pin < PINS_COUNT;
}

bool attachChannel(uint8_t slot, int pin, int frequency, RoboPwmDomain domain) {
    if (domain != ROBOPWM_DOMAIN_SERVO) return false;
    if (slot >= ROBOSERVO_MAX_SERVOS || !isValidPin(pin)) return false;
    if (frequency < ROBOSERVO_MIN_FREQUENCY || frequency > ROBOSERVO_MAX_FREQUENCY) return false;

    _frameFrequency = frequency;
    detachChannel(slot);

    PinName pinName = digitalPinToPinName(pin);
    if (pinName == NC) return false;

    MbedServoPulse* channel = new MbedServoPulse(pinName);
    if (channel == nullptr) return false;

    channel->start(1500);
    _channels[slot] = channel;
    return true;
}

void detachChannel(uint8_t slot) {
    if (slot >= ROBOSERVO_MAX_SERVOS) return;
    if (_channels[slot] != nullptr) {
        delete _channels[slot];
        _channels[slot] = nullptr;
    }
}

void writeDuty(uint8_t slot, uint32_t duty, int frequency, uint8_t resolution) {
    if (slot >= ROBOSERVO_MAX_SERVOS || _channels[slot] == nullptr) return;

    if (frequency > 0) {
        _frameFrequency = frequency;
    } else {
        frequency = _frameFrequency;
    }

    const uint32_t pulseUs = dutyToPulseUs(duty, frequency, resolution);
    _channels[slot]->start(pulseUs);
}

}

#endif
