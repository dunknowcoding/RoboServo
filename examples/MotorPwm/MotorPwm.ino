/**
 * MotorPwm.ino - High-frequency PWM motor control
 *
 * Drives a motor enable / PWM input at 20kHz duty cycle.
 * Suitable for TB6612FNG, L298N enable pins, or similar drivers.
 *
 * Wiring: Motor PWM -> GPIO 14, driver logic per module datasheet
 */

#include <RoboMotor.h>

RoboMotor motor;
const int MOTOR_PIN = 14;

void setup() {
    Serial.begin(115200);

    if (motor.attach(MOTOR_PIN) == ROBOMOTOR_INVALID) {
        Serial.println("Failed to attach motor!");
        while (1) delay(1000);
    }

    Serial.printf("Motor attached at %d Hz\n", motor.getFrequency());
    motor.stop();
    delay(500);
}

void loop() {
    // Ramp up 0% -> 100%
    Serial.println("Ramp up");
    for (int duty = 0; duty <= 100; duty += 5) {
        motor.write(duty);
        delay(50);
    }
    delay(500);

    // Ramp down 100% -> 0%
    Serial.println("Ramp down");
    for (int duty = 100; duty >= 0; duty -= 5) {
        motor.write(duty);
        delay(50);
    }
    delay(1000);
}
