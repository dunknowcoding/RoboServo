/**
 * @file RoboPwmBackend.cpp
 * @brief Internal PWM backend for ESP32, ESP8266, nRF52, RP2040, and STM32
 *
 * ESP32: Uses LEDC peripheral for PWM
 * ESP8266: Uses analogWrite with custom frequency
 * ArduinoNRF nRF52: Uses nrfPwmSetPinFrequency + analogWrite (per-pin frequency groups)
 * Generic nRF52 / RP2040 / STM32: Uses analogWrite with shared frequency
 * AVR: Timer1 ISR pulse scheduler (RoboAvrBackend)
 * Renesas UNO R4: FSP PWM with microsecond period (RoboRenesasBackend)
 * Mbed: 20 ms Ticker frame (RoboMbedBackend)
 * Zephyr: counter_servo scheduler (RoboZephyrBackend)
 */

#include "RoboPlatform.h"
#include "RoboServo.h"
#include "RoboPwmBackend.h"

#if defined(ROBOSERVO_PLATFORM_AVR)
    #include "RoboAvrBackend.h"
#endif
#if defined(ROBOSERVO_PLATFORM_RENESAS)
    #include "RoboRenesasBackend.h"
#endif
#if defined(ROBOSERVO_PLATFORM_MBED)
    #include "RoboMbedBackend.h"
#endif
#if defined(ROBOSERVO_PLATFORM_ZEPHYR)
    #include "RoboZephyrBackend.h"
#endif

#if defined(ROBOSERVO_PLATFORM_ESP32)
    #include "driver/ledc.h"
    #include "esp32-hal-ledc.h"
#endif

// =============================================================================
// Shared Pin Registry
// =============================================================================

static uint64_t _usedPinMask = 0;

#if defined(ROBOSERVO_PLATFORM_ESP8266) \
 || defined(ROBOSERVO_PLATFORM_NRF52_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_NRF53_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_RP2040) \
 || defined(ROBOSERVO_PLATFORM_STM32)
static int _analogGlobalFreq = 0;
#endif

#if defined(ROBOSERVO_PLATFORM_ESP8266) \
 || defined(ROBOSERVO_PLATFORM_NRF52_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_NRF53_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_RP2040) \
 || defined(ROBOSERVO_PLATFORM_STM32)
namespace {

bool setAnalogFrequency(int frequency) {
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    analogWriteFreq(frequency);
    return true;
#elif defined(analogWriteFrequency)
    return analogWriteFrequency((uint32_t)frequency);
#elif defined(analogWritePeriodUs)
    if (frequency <= 0) return false;
    return analogWritePeriodUs(1000000UL / (uint32_t)frequency);
#else
    (void)frequency;
    return true;
#endif
}

} // namespace
#endif

namespace RoboPwmBackend {

bool isValidPwmPin(int pin) {
    if (pin < 0) return false;

#if defined(ROBOSERVO_PLATFORM_ESP8266)
    return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 16);
#elif defined(ROBOSERVO_PLATFORM_NRF52_ARDUINONRF)
    return nrfDigitalPinHasPwm((uint8_t)pin);
#elif defined(ROBOSERVO_PLATFORM_NRF52_GENERIC) \
   || defined(ROBOSERVO_PLATFORM_NRF53_GENERIC) \
   || defined(ROBOSERVO_PLATFORM_RP2040) \
   || defined(ROBOSERVO_PLATFORM_STM32)
    #ifdef digitalPinHasPWM
    return digitalPinHasPWM(pin);
    #else
    return pin >= 0 && pin < 48;
    #endif
#elif defined(ROBOSERVO_PLATFORM_RENESAS)
    return RoboRenesasBackend::isValidPin(pin);
#elif defined(ROBOSERVO_PLATFORM_MBED)
    return RoboMbedBackend::isValidPin(pin);
#elif defined(ROBOSERVO_PLATFORM_ZEPHYR)
    return RoboZephyrBackend::isSupported() && RoboZephyrBackend::isValidPin(pin);
#elif defined(ROBOSERVO_PLATFORM_AVR)
    return RoboAvrBackend::isValidPin(pin);
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
#if defined(ROBOSERVO_PLATFORM_AVR)
    (void)resolution;
    return RoboAvrBackend::attachChannel(hwChannel, pin, frequency, domain);
#elif defined(ROBOSERVO_PLATFORM_RENESAS)
    (void)resolution;
    return RoboRenesasBackend::attachChannel(hwChannel, pin, frequency, domain);
#elif defined(ROBOSERVO_PLATFORM_MBED)
    (void)resolution;
    return RoboMbedBackend::attachChannel(hwChannel, pin, frequency, domain);
#elif defined(ROBOSERVO_PLATFORM_ZEPHYR)
    (void)resolution;
    return RoboZephyrBackend::attachChannel(hwChannel, pin, frequency, domain);
#elif defined(ROBOSERVO_PLATFORM_ESP8266) \
 || defined(ROBOSERVO_PLATFORM_NRF52_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_NRF53_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_RP2040) \
 || defined(ROBOSERVO_PLATFORM_STM32)
    (void)hwChannel;
    (void)domain;
    (void)resolution;
    if (_analogGlobalFreq != frequency) {
        if (!setAnalogFrequency(frequency)) return false;
        _analogGlobalFreq = frequency;
    }
    #ifdef analogWriteResolution
    analogWriteResolution(resolution);
    #endif
    pinMode(pin, OUTPUT);
    return true;
#elif defined(ROBOSERVO_PLATFORM_NRF52_ARDUINONRF)
    (void)hwChannel;
    (void)domain;
    if (!nrfPwmSetPinFrequency((uint8_t)pin, (uint32_t)frequency)) {
        return false;
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
#if defined(ROBOSERVO_PLATFORM_AVR)
    (void)pin;
    (void)domain;
    RoboAvrBackend::detachChannel(hwChannel);
#elif defined(ROBOSERVO_PLATFORM_RENESAS)
    (void)pin;
    (void)domain;
    RoboRenesasBackend::detachChannel(hwChannel);
#elif defined(ROBOSERVO_PLATFORM_MBED)
    (void)pin;
    (void)domain;
    RoboMbedBackend::detachChannel(hwChannel);
#elif defined(ROBOSERVO_PLATFORM_ZEPHYR)
    (void)pin;
    (void)domain;
    RoboZephyrBackend::detachChannel(hwChannel);
#elif defined(ROBOSERVO_USE_PWM_BACKEND)
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
#if defined(ROBOSERVO_PLATFORM_AVR)
    (void)pin;
    (void)domain;
    RoboAvrBackend::writeDuty(hwChannel, duty, 0, ROBOSERVO_PWM_RESOLUTION);
#elif defined(ROBOSERVO_PLATFORM_RENESAS)
    (void)pin;
    (void)domain;
    RoboRenesasBackend::writeDuty(hwChannel, duty, 0, ROBOSERVO_PWM_RESOLUTION);
#elif defined(ROBOSERVO_PLATFORM_MBED)
    (void)pin;
    (void)domain;
    RoboMbedBackend::writeDuty(hwChannel, duty, 0, ROBOSERVO_PWM_RESOLUTION);
#elif defined(ROBOSERVO_PLATFORM_ZEPHYR)
    (void)pin;
    (void)domain;
    RoboZephyrBackend::writeDuty(hwChannel, duty, 0, ROBOSERVO_PWM_RESOLUTION);
#elif defined(ROBOSERVO_USE_PWM_BACKEND)
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
