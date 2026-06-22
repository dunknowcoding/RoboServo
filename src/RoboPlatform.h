/**
 * @file RoboPlatform.h
 * @brief Shared platform detection for RoboServo and RoboMotor
 *
 * Tier 0 — ESP32 / ESP8266 (native LEDC / analogWriteFreq)
 * Tier 1 — ArduinoNRF nRF52 (per-pin nrfPwmSetPinFrequency)
 * Tier 1b — Generic nRF52 (Adafruit / Nordic DK cores)
 * Tier 2 — RP2040 / STM32 / nRF53 generic (shared analogWrite backend)
 * Tier 3 — AVR (Timer1 ISR, limited channels)
 * Tier 4 — Renesas UNO R4 (FSP PWM, microsecond period)
 * Tier 5 — Mbed (Ticker + DigitalOut, 20 ms frame)
 * Tier 6 — Zephyr (counter_servo bit-bang, UNO Q and compatible boards)
 */

#ifndef ROBOPLATFORM_H
#define ROBOPLATFORM_H

#include <Arduino.h>

// =============================================================================
// Platform Detection
// =============================================================================

#if defined(ESP8266)
    #define ROBOSERVO_PLATFORM_ESP8266
#elif defined(ESP32)
    #define ROBOSERVO_PLATFORM_ESP32
#elif defined(ARDUINO_ARCH_NRF53) \
   || defined(NRF5340_XXAA_APPLICATION) \
   || defined(NRF5340_XXAA) \
   || defined(ARDUINO_NRF5340_XXAA)
    #if defined(ARDUINO_ARCH_MBED)
        #define ROBOSERVO_PLATFORM_MBED
    #elif defined(ARDUINO_ARCH_ZEPHYR)
        #define ROBOSERVO_PLATFORM_ZEPHYR
    #elif defined(ARDUINO_NRF52_PROMICRO) \
     || defined(ARDUINO_NRF52_NICENANO_V2) \
     || defined(ARDUINO_NRF52_SUPERMINI) \
     || defined(ARDUINO_NRF52_NRFMICRO) \
     || defined(ARDUINO_NRF52_MINI) \
     || defined(ARDUINO_NRF52_XIAO) \
     || defined(ARDUINO_NRF52_DEVBOARD) \
     || defined(ARDUINO_NRF52_DEVBOARD_833) \
     || defined(ARDUINO_NRF52_PITAYA_GO) \
     || defined(ARDUINO_NRF52_USB_DONGLE) \
     || defined(NRF_BOARD_HAS_NATIVE_USB) \
     || defined(NRF_BOARD_HAS_BATTERY)
        #define ROBOSERVO_PLATFORM_NRF52_ARDUINONRF
    #else
        #define ROBOSERVO_PLATFORM_NRF53_GENERIC
    #endif
#elif defined(ARDUINO_ARCH_NRF52)
    #if defined(ARDUINO_NRF52_PROMICRO) \
     || defined(ARDUINO_NRF52_NICENANO_V2) \
     || defined(ARDUINO_NRF52_SUPERMINI) \
     || defined(ARDUINO_NRF52_NRFMICRO) \
     || defined(ARDUINO_NRF52_MINI) \
     || defined(ARDUINO_NRF52_XIAO) \
     || defined(ARDUINO_NRF52_DEVBOARD) \
     || defined(ARDUINO_NRF52_DEVBOARD_833) \
     || defined(ARDUINO_NRF52_PITAYA_GO) \
     || defined(ARDUINO_NRF52_USB_DONGLE) \
     || defined(NRF_BOARD_HAS_NATIVE_USB) \
     || defined(NRF_BOARD_HAS_BATTERY)
        #define ROBOSERVO_PLATFORM_NRF52_ARDUINONRF
    #else
        #define ROBOSERVO_PLATFORM_NRF52_GENERIC
    #endif
#elif defined(ARDUINO_ARCH_RENESAS)
    #define ROBOSERVO_PLATFORM_RENESAS
#elif defined(ARDUINO_ARCH_MBED)
    #define ROBOSERVO_PLATFORM_MBED
#elif defined(ARDUINO_ARCH_ZEPHYR)
    #define ROBOSERVO_PLATFORM_ZEPHYR
#elif defined(ARDUINO_ARCH_RP2040)
    #define ROBOSERVO_PLATFORM_RP2040
#elif defined(ARDUINO_ARCH_STM32)
    #define ROBOSERVO_PLATFORM_STM32
#elif defined(ARDUINO_ARCH_AVR)
    #define ROBOSERVO_PLATFORM_AVR
#else
    #error "RoboServo library does not support this board"
#endif

// =============================================================================
// Backend Routing
// =============================================================================

#if defined(ROBOSERVO_PLATFORM_ESP8266) \
 || defined(ROBOSERVO_PLATFORM_NRF52_ARDUINONRF) \
 || defined(ROBOSERVO_PLATFORM_NRF52_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_NRF53_GENERIC) \
 || defined(ROBOSERVO_PLATFORM_RP2040) \
 || defined(ROBOSERVO_PLATFORM_STM32)
    #define ROBOSERVO_USE_PWM_BACKEND
#endif

#if defined(ROBOSERVO_PLATFORM_AVR)
    #define ROBOSERVO_USE_AVR_BACKEND
#endif

#if defined(ROBOSERVO_PLATFORM_RENESAS)
    #define ROBOSERVO_USE_RENESAS_BACKEND
#endif

#if defined(ROBOSERVO_PLATFORM_MBED)
    #define ROBOSERVO_USE_MBED_BACKEND
#endif

#if defined(ROBOSERVO_PLATFORM_ZEPHYR)
    #define ROBOSERVO_USE_ZEPHYR_BACKEND
#endif

#if defined(ROBOSERVO_USE_PWM_BACKEND) \
 || defined(ROBOSERVO_USE_AVR_BACKEND) \
 || defined(ROBOSERVO_USE_RENESAS_BACKEND) \
 || defined(ROBOSERVO_USE_MBED_BACKEND) \
 || defined(ROBOSERVO_USE_ZEPHYR_BACKEND)
    #define ROBOSERVO_USE_HW_BACKEND
#endif

#endif // ROBOPLATFORM_H
