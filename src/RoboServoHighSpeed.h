/**
 * @file RoboServoHighSpeed.h
 * @brief Helper presets for high refresh-rate PWM servos (300-400Hz)
 *
 * High-speed digital servos use the same microsecond pulse protocol as standard
 * servos but accept a higher PWM refresh rate for faster response.
 *
 * Usage:
 * @code
 *   #include <RoboServoHighSpeed.h>
 *
 *   RoboServo servo;
 *   roboServoAttachHighSpeed(servo, 13);
 *   servo.write(90);
 * @endcode
 */

#ifndef ROBOSERVO_HIGHSPEED_H
#define ROBOSERVO_HIGHSPEED_H

#include "RoboServo.h"

/** Common high refresh-rate for digital servos (Hz) */
#define ROBOSERVO_HIGH_SPEED_FREQUENCY 333

/** Alternative high refresh-rate (Hz) */
#define ROBOSERVO_HIGH_SPEED_FREQUENCY_ALT 400

/**
 * @brief Attach a high refresh-rate servo with default pulse range (500-2500us)
 * @return Channel number, or ROBOSERVO_INVALID_SERVO (255) on failure
 */
inline uint8_t roboServoAttachHighSpeed(RoboServo& servo, int pin) {
    return servo.attach(pin, ROBOSERVO_DEFAULT_MIN_PULSE_US, ROBOSERVO_DEFAULT_MAX_PULSE_US,
                        SERVO_TYPE_180, ROBOSERVO_HIGH_SPEED_FREQUENCY);
}

/**
 * @brief Attach a high refresh-rate servo with custom pulse range
 * @return Channel number, or ROBOSERVO_INVALID_SERVO (255) on failure
 */
inline uint8_t roboServoAttachHighSpeed(RoboServo& servo, int pin, int minPulseUs, int maxPulseUs) {
    return servo.attach(pin, minPulseUs, maxPulseUs,
                        SERVO_TYPE_180, ROBOSERVO_HIGH_SPEED_FREQUENCY);
}

#endif // ROBOSERVO_HIGHSPEED_H
