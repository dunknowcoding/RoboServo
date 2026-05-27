/**
 * @file RoboPwmBackend.cpp
 * @brief Internal PWM backend for ESP32 and ESP8266
 *
 * ESP32: Uses LEDC peripheral for PWM
 * ESP8266: Uses analogWrite with custom frequency
 */

// Platform detection must precede the header (conditional declarations)
#if defined(ESP8266)
    #define ROBOSERVO_PLATFORM_ESP8266
#elif defined(ESP32)
    #define ROBOSERVO_PLATFORM_ESP32
#endif

#include "RoboPwmBackend.h"

#if defined(ROBOSERVO_PLATFORM_ESP32)
    #include "driver/ledc.h"
    #include "esp32-hal-ledc.h"
#endif

// =============================================================================
// Platform Detection (mirror RoboServo.h)
// =============================================================================

// =============================================================================
// Shared Pin Registry
// =============================================================================

static uint64_t _usedPinMask = 0;

#if defined(ROBOSERVO_PLATFORM_ESP8266)
static int _esp8266GlobalFreq = 0;
#endif

namespace RoboPwmBackend {

bool isValidPwmPin(int pin) {
    if (pin < 0) return false;

#if defined(ROBOSERVO_PLATFORM_ESP8266)
    return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 16);
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    return (pin >= 0 && pin <= 54) && (pin != 24) && (pin != 25);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    return (pin >= 1 && pin <= 21) || (pin >= 35 && pin <= 45) || (pin == 47) || (pin == 48);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    return (pin >= 1 && pin <= 21) || (pin == 26) || (pin >= 33 && pin <= 42);
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    return (pin >= 0 && pin <= 10) || (pin >= 18 && pin <= 21);
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    return (pin >= 0 && pin <= 23);
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
    return (pin >= 0 && pin <= 14) || (pin >= 25 && pin <= 27);
#elif defined(CONFIG_IDF_TARGET_ESP32)
    return (pin == 2) || (pin == 4) || (pin == 5) || (pin >= 12 && pin <= 19) ||
           (pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) || (pin == 32) || (pin == 33);
#else
    return (pin >= 0 && pin <= 48);
#endif
}

bool isPinInUse(int pin) {
    if (pin < 0 || pin >= 64) return false;
    return (_usedPinMask & (1ULL << pin)) != 0;
}

void markPinUsed(int pin) {
    if (pin >= 0 && pin < 64) _usedPinMask |= (1ULL << pin);
}

void markPinFree(int pin) {
    if (pin >= 0 && pin < 64) _usedPinMask &= ~(1ULL << pin);
}

bool attachPin(int pin, uint8_t hwChannel, int frequency, uint8_t resolution, RoboPwmDomain domain) {
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    (void)hwChannel;
    (void)domain;
    if (_esp8266GlobalFreq != frequency) {
        analogWriteFreq(frequency);
        _esp8266GlobalFreq = frequency;
    }
    analogWriteResolution(resolution);
    pinMode(pin, OUTPUT);
    return true;
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // Motor domain: explicit channel keeps kHz PWM isolated from auto-assigned servo channels
    if (domain == ROBOPWM_DOMAIN_MOTOR) {
        return ledcAttachChannel(pin, (uint32_t)frequency, resolution, (int8_t)hwChannel);
    }
    return ledcAttach(pin, (uint32_t)frequency, resolution);
#else
    (void)domain;
    double actualFreq = ledcSetup(hwChannel, (double)frequency, resolution);
    if (actualFreq == 0) return false;
    ledcAttachPin(pin, hwChannel);
    return true;
#endif
}

void detachPin(int pin, uint8_t hwChannel, RoboPwmDomain domain) {
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    (void)hwChannel;
    (void)domain;
    analogWrite(pin, 0);
    pinMode(pin, INPUT);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)hwChannel;
    (void)domain;
    ledcWrite(pin, 0);
    ledcDetach(pin);
#else
    (void)domain;
    ledcWrite(hwChannel, 0);
    ledcDetachPin(pin);
#endif
}

void writeDuty(int pin, uint8_t hwChannel, uint32_t duty, RoboPwmDomain domain) {
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    (void)hwChannel;
    (void)domain;
    analogWrite(pin, duty);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)hwChannel;
    (void)domain;
    ledcWrite(pin, duty);
#else
    (void)domain;
    ledcWrite(hwChannel, duty);
#endif
}

uint32_t microsecondsToTicks(int us, int frequency, uint8_t resolution) {
    uint32_t maxDuty = (1UL << resolution) - 1;
    uint32_t periodUs = 1000000UL / (uint32_t)frequency;
    return (uint32_t)(((uint64_t)us * maxDuty) / periodUs);
}

uint32_t percentToTicks(int percent, uint8_t resolution) {
    if (percent < 0) percent = 0;
    else if (percent > 100) percent = 100;
    uint32_t maxDuty = (1UL << resolution) - 1;
    return (uint32_t)(((uint64_t)percent * maxDuty) / 100);
}

}
