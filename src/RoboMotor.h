/**
 * @file RoboMotor.h
 * @brief High-frequency PWM motor control for ESP32 family and ESP8266
 *
 * @author dunknowcoding
 * @version 1.2.0
 * @license MIT
 *
 * Supported boards: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2, ESP32-P4, ESP8266
 *
 * Features:
 *   - kHz-range PWM for motor drivers and ESC enable inputs
 *   - Duty cycle control (0-100%)
 *   - Separate LEDC channel pool from RoboServo (timer isolation on ESP32)
 *   - Grouped motor control
 *   - Shared GPIO registry with RoboServo
 */

#ifndef ROBOMOTOR_H
#define ROBOMOTOR_H

#include <Arduino.h>
#include "RoboPwmBackend.h"

// =============================================================================
// Platform Detection
// =============================================================================

#if defined(ESP8266)
    #define ROBOMOTOR_PLATFORM_ESP8266
#elif defined(ESP32)
    #define ROBOMOTOR_PLATFORM_ESP32
#else
    #error "RoboMotor library only supports ESP32 and ESP8266 boards"
#endif

// =============================================================================
// Configuration Constants
// =============================================================================

/** Motor channel pool — uses upper LEDC channels to avoid 50Hz servo timers (ESP32) */
#if defined(ROBOMOTOR_PLATFORM_ESP8266)
    #define ROBOMOTOR_MAX_MOTORS      4
    #define ROBOMOTOR_CHANNEL_BASE    0
    #define ROBOMOTOR_PWM_RESOLUTION  10
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
    #define ROBOMOTOR_MAX_MOTORS      2
    #define ROBOMOTOR_CHANNEL_BASE    4
    #define ROBOMOTOR_PWM_RESOLUTION  10
#elif defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3)
    #define ROBOMOTOR_MAX_MOTORS      4
    #define ROBOMOTOR_CHANNEL_BASE    4
    #define ROBOMOTOR_PWM_RESOLUTION  10
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    #define ROBOMOTOR_MAX_MOTORS      4
    #define ROBOMOTOR_CHANNEL_BASE    8
    #define ROBOMOTOR_PWM_RESOLUTION  10
#elif defined(CONFIG_IDF_TARGET_ESP32)
    #define ROBOMOTOR_MAX_MOTORS      4
    #define ROBOMOTOR_CHANNEL_BASE    8
    #define ROBOMOTOR_PWM_RESOLUTION  10
#else
    #define ROBOMOTOR_MAX_MOTORS      2
    #define ROBOMOTOR_CHANNEL_BASE    4
    #define ROBOMOTOR_PWM_RESOLUTION  10
#endif

/** Return value when motor attachment fails */
#define ROBOMOTOR_INVALID 255

/** PWM frequency settings for motor / ESC enable outputs */
#define ROBOMOTOR_DEFAULT_FREQUENCY 20000  ///< Default 20kHz (audible-free PWM)
#define ROBOMOTOR_MIN_FREQUENCY     1000   ///< Minimum allowed frequency (Hz)
#define ROBOMOTOR_MAX_FREQUENCY     40000  ///< Maximum allowed frequency (Hz)

// =============================================================================
// RoboMotor Class
// =============================================================================

/**
 * @class RoboMotor
 * @brief Controls a motor via high-frequency PWM duty cycle
 *
 * Basic usage:
 * @code
 *   RoboMotor motor;
 *   motor.attach(14);       // Attach to GPIO 14 at 20kHz
 *   motor.write(50);        // 50% duty cycle
 *   motor.stop();           // 0% duty (coast / off)
 * @endcode
 *
 * For ESC or motor driver enable pins, connect the PWM pin to the EN / PWM input.
 */
class RoboMotor {
public:
    RoboMotor();
    ~RoboMotor();

    // -------------------------------------------------------------------------
    // Attachment Methods
    // -------------------------------------------------------------------------

    /** @brief Attach motor with default settings (20kHz, 10-bit) */
    uint8_t attach(int pin);

    /** @brief Attach motor with custom frequency */
    uint8_t attach(int pin, int frequency);

    /** @brief Attach motor with custom frequency and resolution */
    uint8_t attach(int pin, int frequency, uint8_t resolution);

    /** @brief Detach motor and release PWM channel */
    void detach();

    /** @brief Check if motor is attached */
    bool attached() const;

    // -------------------------------------------------------------------------
    // Duty Cycle Control
    // -------------------------------------------------------------------------

    /** @brief Set duty cycle as percentage (0 to 100) */
    void write(int dutyPercent);

    /** @brief Set raw duty cycle (0 to 2^resolution - 1) */
    void writeRaw(uint32_t duty);

    /** @brief Get current duty cycle as percentage */
    int read() const;

    /** @brief Get current raw duty cycle */
    uint32_t readRaw() const;

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    void setFrequency(int frequency);
    int getFrequency() const;
    uint8_t getResolution() const;
    int getPin() const;
    uint8_t getChannel() const;

    // -------------------------------------------------------------------------
    // Motor Control
    // -------------------------------------------------------------------------

    /** @brief Stop motor output (0% duty) */
    void stop();

    /** @brief Alias for stop() — set duty to 0% */
    void brake();

    // -------------------------------------------------------------------------
    // Static Utilities
    // -------------------------------------------------------------------------

    static int getAttachedCount();
    static uint32_t getDefaultFrequency() { return ROBOMOTOR_DEFAULT_FREQUENCY; }
    static uint8_t getMotorResolution() { return ROBOMOTOR_PWM_RESOLUTION; }

private:
    int _pin;
    uint8_t _slot;
    uint8_t _hwChannel;
    int _frequency;
    uint8_t _resolution;
    uint32_t _currentDuty;
    int _currentPercent;
    bool _attached;

    uint32_t percentToTicks(int percent) const;
    static uint8_t allocateSlot();
    static void releaseSlot(uint8_t slot);
    static uint8_t slotToHwChannel(uint8_t slot);
};

// =============================================================================
// RoboMotorGroup Class
// =============================================================================

/**
 * @class RoboMotorGroup
 * @brief Manage multiple motors as a coordinated group
 *
 * Usage:
 * @code
 *   RoboMotorGroup group;
 *   group.addMotor(14);
 *   group.addMotor(15);
 *   group.writeAll(50);  // All motors at 50%
 * @endcode
 */
class RoboMotorGroup {
public:
    RoboMotorGroup();
    ~RoboMotorGroup();

    // -------------------------------------------------------------------------
    // Add/Remove Motors
    // -------------------------------------------------------------------------

    int addMotor(int pin);
    int addMotor(int pin, int frequency);
    bool removeMotor(int index);
    void removeAll();

    // -------------------------------------------------------------------------
    // Group Information
    // -------------------------------------------------------------------------

    int count() const;
    RoboMotor* getMotor(int index);

    // -------------------------------------------------------------------------
    // Group Control (all motors)
    // -------------------------------------------------------------------------

    void writeAll(int dutyPercent);
    void writeMultiple(const int* duties, int count);
    void stopAll();
    void detachAll();

    // -------------------------------------------------------------------------
    // Individual Motor Control
    // -------------------------------------------------------------------------

    void write(int index, int dutyPercent);
    int read(int index) const;

private:
    RoboMotor* _motors[ROBOMOTOR_MAX_MOTORS];
    int _count;
};

#endif // ROBOMOTOR_H
