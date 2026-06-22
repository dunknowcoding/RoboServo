/**
 * @file RoboAvrBackend.cpp
 * @brief Timer1 interrupt-driven servo pulses for AVR boards
 *
 * Uses a single 16-bit timer with prescaler 8 (0.5 us/tick at 16 MHz).
 * Supports up to 6 servos on ATmega328-class boards, 12 on Mega.
 */

#include "RoboPlatform.h"

#if defined(ROBOSERVO_PLATFORM_AVR)

#include <avr/interrupt.h>
#include "RoboServo.h"
#include "RoboAvrBackend.h"

struct AvrServoSlot {
    uint8_t pin;
    uint16_t ticks;
    bool active;
};

static AvrServoSlot _slots[ROBOSERVO_MAX_SERVOS];
static volatile int8_t _isrIndex = -1;
static uint16_t _refreshUs = 20000;
static int _frameFrequency = ROBOSERVO_DEFAULT_FREQUENCY;
static bool _timerActive = false;

#define ROBOSERVO_AVR_TRIM_TICKS 2

static inline uint16_t usToTicks(uint16_t us) {
    return (uint16_t)((clockCyclesPerMicrosecond() * (uint32_t)us) / 8U);
}

static uint8_t activeSlotCount() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < ROBOSERVO_MAX_SERVOS; i++) {
        if (_slots[i].active) count++;
    }
    return count;
}

static int8_t findNextActive(int8_t start) {
    for (int8_t i = start; i < (int8_t)ROBOSERVO_MAX_SERVOS; i++) {
        if (_slots[i].active) return i;
    }
    return -1;
}

static void startTimer1() {
    if (_timerActive) return;

    TCCR1A = 0;
    TCCR1B = _BV(CS11);
    TCNT1 = 0;
    _isrIndex = -1;
    OCR1A = usToTicks(_refreshUs);

#if defined(TIFR1) && defined(OCIE1A)
    TIFR1 = _BV(OCF1A);
    TIMSK1 = _BV(OCIE1A);
#elif defined(TIMSK) && defined(OCIE1A)
    TIFR = _BV(OCF1A);
    TIMSK = _BV(OCIE1A);
#endif

    _timerActive = true;
}

static void stopTimer1() {
    if (!_timerActive) return;

#if defined(TIMSK1) && defined(OCIE1A)
    TIMSK1 &= (uint8_t)~_BV(OCIE1A);
#elif defined(TIMSK) && defined(OCIE1A)
    TIMSK &= (uint8_t)~_BV(OCIE1A);
#endif

    for (uint8_t i = 0; i < ROBOSERVO_MAX_SERVOS; i++) {
        if (_slots[i].active) {
            digitalWrite(_slots[i].pin, LOW);
            _slots[i].active = false;
        }
    }

    _timerActive = false;
    _isrIndex = -1;
}

static void handleTimer1Compare() {
    if (_isrIndex >= 0 && _slots[_isrIndex].active) {
        digitalWrite(_slots[_isrIndex].pin, LOW);
    } else if (_isrIndex < 0) {
        TCNT1 = 0;
    }

    int8_t next = (_isrIndex < 0) ? 0 : (_isrIndex + 1);
    next = findNextActive(next);

    if (next >= 0) {
        OCR1A = (uint16_t)(TCNT1 + _slots[next].ticks);
        digitalWrite(_slots[next].pin, HIGH);
        _isrIndex = next;
        return;
    }

    const uint16_t refreshTicks = usToTicks(_refreshUs);
    if ((uint16_t)(TCNT1 + 4U) < refreshTicks) {
        OCR1A = refreshTicks;
    } else {
        OCR1A = (uint16_t)(TCNT1 + 4U);
    }
    _isrIndex = -1;
}

ISR(TIMER1_COMPA_vect) {
    handleTimer1Compare();
}

namespace RoboAvrBackend {

bool isValidPin(int pin) {
    return (pin >= 0 && pin < NUM_DIGITAL_PINS);
}

bool attachChannel(uint8_t slot, int pin, int frequency, RoboPwmDomain domain) {
    if (domain != ROBOPWM_DOMAIN_SERVO) return false;
    if (slot >= ROBOSERVO_MAX_SERVOS || !isValidPin(pin)) return false;
    if (frequency < ROBOSERVO_MIN_FREQUENCY || frequency > ROBOSERVO_MAX_FREQUENCY) return false;

    _frameFrequency = frequency;
    _refreshUs = (uint16_t)(1000000UL / (uint32_t)frequency);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    uint8_t oldSREG = SREG;
    cli();
    _slots[slot].pin = (uint8_t)pin;
    _slots[slot].ticks = usToTicks(1500U - ROBOSERVO_AVR_TRIM_TICKS);
    _slots[slot].active = true;
    SREG = oldSREG;

    startTimer1();
    return true;
}

void detachChannel(uint8_t slot) {
    if (slot >= ROBOSERVO_MAX_SERVOS) return;

    uint8_t oldSREG = SREG;
    cli();
    if (_slots[slot].active) {
        digitalWrite(_slots[slot].pin, LOW);
        _slots[slot].active = false;
    }
    SREG = oldSREG;

    if (activeSlotCount() == 0) {
        stopTimer1();
    }
}

void writeDuty(uint8_t slot, uint32_t duty, int frequency, uint8_t resolution) {
    if (slot >= ROBOSERVO_MAX_SERVOS || !_slots[slot].active) return;

    if (frequency > 0) {
        _frameFrequency = frequency;
        _refreshUs = (uint16_t)(1000000UL / (uint32_t)frequency);
    } else {
        frequency = _frameFrequency;
    }

    const uint32_t maxDuty = (1UL << resolution) - 1UL;
    const uint32_t periodUs = (frequency > 0) ? (1000000UL / (uint32_t)frequency) : _refreshUs;
    uint32_t pulseUs = (maxDuty > 0) ? ((duty * periodUs) / maxDuty) : 0U;
    if (pulseUs > 0xFFFFU) pulseUs = 0xFFFFU;

    uint16_t ticks = usToTicks((uint16_t)pulseUs);
    if (ticks > ROBOSERVO_AVR_TRIM_TICKS) {
        ticks = (uint16_t)(ticks - ROBOSERVO_AVR_TRIM_TICKS);
    } else {
        ticks = 1;
    }

    uint8_t oldSREG = SREG;
    cli();
    _slots[slot].ticks = ticks;
    SREG = oldSREG;
}

}

#endif // ROBOSERVO_PLATFORM_AVR
