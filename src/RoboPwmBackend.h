/**
 * @file RoboPwmBackend.h
 * @brief Internal PWM backend shared by RoboServo and RoboMotor
 *
 * Manages GPIO pin ownership and platform-specific LEDC / analogWrite output.
 * Not part of the public API — include RoboServo.h or RoboMotor.h instead.
 */

#ifndef ROBOPWMBACKEND_H
#define ROBOPWMBACKEND_H

#include <Arduino.h>

// =============================================================================
// Domain Enumeration
// =============================================================================

/** PWM output domain — selects channel pool and attach strategy */
enum RoboPwmDomain {
    ROBOPWM_DOMAIN_SERVO = 0,
    ROBOPWM_DOMAIN_MOTOR = 1
};

// =============================================================================
// RoboPwmBackend Namespace
// =============================================================================

namespace RoboPwmBackend {

    // -------------------------------------------------------------------------
    // GPIO Pin Registry (shared across all domains)
    // -------------------------------------------------------------------------

    bool isValidPwmPin(int pin);
    bool isPinInUse(int pin);
    void markPinUsed(int pin);
    void markPinFree(int pin);

    // -------------------------------------------------------------------------
    // Platform PWM Attach / Detach / Write
    // -------------------------------------------------------------------------

    /**
     * @brief Attach a pin to PWM hardware
     * @param pin GPIO pin number
     * @param hwChannel LEDC channel (ESP32) or logical index (ESP8266)
     * @param frequency PWM frequency in Hz
     * @param resolution PWM resolution in bits
     * @param domain Servo or motor domain (selects Core 3.x attach strategy)
     * @return true on success
     */
    bool attachPin(int pin, uint8_t hwChannel, int frequency, uint8_t resolution, RoboPwmDomain domain);

    /** @brief Detach pin from PWM hardware */
    void detachPin(int pin, uint8_t hwChannel, RoboPwmDomain domain);

    /** @brief Write raw duty cycle to pin */
    void writeDuty(int pin, uint8_t hwChannel, uint32_t duty, RoboPwmDomain domain);

    /** @brief Convert microseconds to duty ticks at given frequency and resolution */
    uint32_t microsecondsToTicks(int us, int frequency, uint8_t resolution);

    /** @brief Convert duty percentage (0-100) to duty ticks */
    uint32_t percentToTicks(int percent, uint8_t resolution);

}

#endif // ROBOPWMBACKEND_H
