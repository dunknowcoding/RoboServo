/**
 * ServoAndMotor.ino - Low-speed servo and high-speed motor on the same board
 *
 * Demonstrates timer isolation: 50Hz servo + 20kHz motor PWM coexist on ESP32.
 * Servo: GPIO 13, Motor enable: GPIO 14
 *
 * Note: ESP8266 cannot mix 50Hz and 20kHz — use RoboServo or RoboMotor alone.
 */

#include <RoboServo.h>
#include <RoboMotor.h>

RoboServo servo;
RoboMotor motor;
const int SERVO_PIN = 13;
const int MOTOR_PIN = 14;

void setup() {
    Serial.begin(115200);

    if (servo.attach(SERVO_PIN) == ROBOSERVO_INVALID_SERVO) {
        Serial.println("Failed to attach servo!");
        while (1) delay(1000);
    }
    if (motor.attach(MOTOR_PIN) == ROBOMOTOR_INVALID) {
        Serial.println("Failed to attach motor!");
        while (1) delay(1000);
    }

    servo.write(90);
    motor.stop();
    Serial.println("Servo (50Hz) + Motor (20kHz) ready");
}

void loop() {
    // Sweep servo while motor runs at half duty
    Serial.println("Servo sweep + motor at 50%");
    motor.write(50);
    for (int angle = 0; angle <= 180; angle += 10) {
        servo.write(angle);
        delay(100);
    }
    for (int angle = 180; angle >= 0; angle -= 10) {
        servo.write(angle);
        delay(100);
    }
    motor.stop();
    delay(500);

    // Pulse motor while servo holds center
    Serial.println("Motor pulse + servo at 90°");
    servo.write(90);
    for (int duty = 0; duty <= 80; duty += 10) {
        motor.write(duty);
        delay(200);
    }
    for (int duty = 80; duty >= 0; duty -= 10) {
        motor.write(duty);
        delay(200);
    }
    delay(1000);
}
