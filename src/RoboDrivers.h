/**
 * @file RoboDrivers.h
 * @brief Safe, profile-aware control for common servo and motor driver boards.
 *
 * Copyright 2026 dunknowcoding
 * SPDX-License-Identifier: MIT
 */

#ifndef ROBODRIVERS_H
#define ROBODRIVERS_H

#include <Arduino.h>
#include <Wire.h>

#define ROBO_DRIVER_PIN_NONE (-1)

enum RoboDriverModule {
    ROBO_DRIVER_L298N = 0,
    ROBO_DRIVER_L293D,
    ROBO_DRIVER_L298P_SHIELD,
    ROBO_DRIVER_L293D_SHIELD,
    ROBO_DRIVER_TB6612,
    ROBO_DRIVER_DRV8833_RED,
    ROBO_DRIVER_DRV8833_BLACK,
    ROBO_DRIVER_DRV8871,
    ROBO_DRIVER_BTS7960,
    ROBO_DRIVER_ULN2003,
    ROBO_DRIVER_A4988,
    ROBO_DRIVER_DRV8825,
    ROBO_DRIVER_TMC2208,
    ROBO_DRIVER_TMC2209,
    ROBO_DRIVER_ATD5833,
    ROBO_DRIVER_TB6600,
    ROBO_DRIVER_1803BK,
    ROBO_DRIVER_PCA9685,
    ROBO_DRIVER_MODULE_COUNT
};

enum RoboDriverKind {
    ROBO_DRIVER_HBRIDGE = 0,
    ROBO_DRIVER_STEP_DIR,
    ROBO_DRIVER_FOUR_WIRE_STEPPER,
    ROBO_DRIVER_PWM_EXPANDER,
    ROBO_DRIVER_MANUAL_ONLY
};

struct RoboDriverInfo {
    RoboDriverModule module;
    const char *id;
    const char *name;
    RoboDriverKind kind;
    uint8_t channels;
    uint8_t maximumMicrosteps;
    uint16_t minimumStepPulseUs;
};

const RoboDriverInfo *roboDriverInfo(RoboDriverModule module);

/** One DC motor channel on an H-bridge. */
class RoboHBridge {
public:
    RoboHBridge();

    bool beginIn1In2(RoboDriverModule module, int in1, int in2,
                     int pwm = ROBO_DRIVER_PIN_NONE,
                     int standby = ROBO_DRIVER_PIN_NONE);
    bool beginPwmDirection(RoboDriverModule module, int pwm, int direction,
                           int brake = ROBO_DRIVER_PIN_NONE);
    bool beginDualPwm(RoboDriverModule module, int forwardPwm, int reversePwm,
                      int forwardEnable, int reverseEnable);

    void drive(int signedPercent);
    void coast();
    void brake();
    void setEnabled(bool enabled);
    bool attached() const;
    int read() const;
    RoboDriverModule module() const;

private:
    enum Mode { MODE_NONE = 0, MODE_IN1_IN2, MODE_PWM_DIR, MODE_DUAL_PWM };
    RoboDriverModule _module;
    Mode _mode;
    int _a;
    int _b;
    int _pwm;
    int _enableA;
    int _enableB;
    int _value;
    bool _enabled;

    static void pwmWritePercent(int pin, int percent);
};

/** STEP/DIR driver with module-specific microstep truth tables. */
class RoboStepDir {
public:
    RoboStepDir();

    bool begin(RoboDriverModule module, int stepPin, int directionPin,
               int enablePin = ROBO_DRIVER_PIN_NONE,
               int ms1 = ROBO_DRIVER_PIN_NONE,
               int ms2 = ROBO_DRIVER_PIN_NONE,
               int ms3 = ROBO_DRIVER_PIN_NONE,
               int resetPin = ROBO_DRIVER_PIN_NONE,
               int sleepPin = ROBO_DRIVER_PIN_NONE,
               bool controlActiveLow = true);
    bool setMicrosteps(uint8_t microsteps);
    bool confirmExternalMicrosteps(uint8_t microsteps);
    void setEnabled(bool enabled);
    void sleep(bool sleeping);
    void resetIndexer();
    bool step(bool forward);
    bool step(bool forward, uint16_t pulseUs);
    bool attached() const;
    uint8_t microsteps() const;
    uint16_t minimumPulseUs() const;
    RoboDriverModule module() const;

private:
    RoboDriverModule _module;
    int _step;
    int _direction;
    int _enable;
    int _ms[3];
    int _reset;
    int _sleep;
    uint8_t _microsteps;
    uint16_t _minimumPulseUs;
    bool _activeLow;
    bool _attached;

    bool setBinaryMode(uint8_t a, uint8_t b, uint8_t c);
    bool setTriStateMode(int8_t a, int8_t b);
};

/** Four-coil half-step driver, including the ULN2003/28BYJ-48 combination. */
class RoboFourWireStepper {
public:
    RoboFourWireStepper();
    bool begin(RoboDriverModule module, int in1, int in2, int in3, int in4);
    bool step(bool forward);
    void release();
    long position() const;
    bool attached() const;

private:
    RoboDriverModule _module;
    int _pins[4];
    uint8_t _phase;
    long _position;
    bool _attached;
    void applyPhase();
};

/** Fixed-routing control for the common two-L293D, 74HC595 motor shield. */
class RoboL293DShield {
public:
    RoboL293DShield();
    bool begin(int dataPin = 8, int latchPin = 12,
               int clockPin = 4, int outputEnablePin = 7);
    bool drive(uint8_t channel, int signedPercent);
    bool coast(uint8_t channel);
    bool brake(uint8_t channel);
    void releaseAll();

private:
    int _data;
    int _latch;
    int _clock;
    int _oe;
    uint8_t _state;
    bool _attached;
    void commit();
    bool setDirection(uint8_t channel, int direction, bool braking);
};

/** Bounded 16-channel PCA9685 servo-signal output. */
class RoboPca9685 {
public:
    RoboPca9685();
    bool begin(TwoWire &wire = Wire, uint8_t address = 0x40);
    bool setFrequency(uint16_t hz);
    bool writeTicks(uint8_t channel, uint16_t onTick, uint16_t offTick);
    bool writeMicroseconds(uint8_t channel, uint16_t pulseUs);
    void allOff();
    uint16_t frequency() const;

private:
    TwoWire *_wire;
    uint8_t _address;
    uint16_t _frequency;
    bool _attached;
    bool write8(uint8_t reg, uint8_t value);
    bool read8(uint8_t reg, uint8_t &value);
};

#endif
