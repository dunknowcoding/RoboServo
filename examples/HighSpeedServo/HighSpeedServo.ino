/**
 * HighSpeedServo.ino - High refresh-rate servo control (333Hz)
 *
 * Digital servos that support 300-400Hz PWM can respond faster than
 * standard 50Hz servos while using the same pulse width protocol.
 *
 * Wiring: Servo signal -> GPIO 13, VCC -> 5V, GND -> GND
 */

#include <RoboServoHighSpeed.h>

RoboServo myServo;
const int SERVO_PIN = 13;

void setup() {
    Serial.begin(115200);

    if (roboServoAttachHighSpeed(myServo, SERVO_PIN) == ROBOSERVO_INVALID_SERVO) {
        Serial.println("Failed to attach servo!");
        while (1) delay(1000);
    }

    Serial.printf("High-speed servo attached at %d Hz\n", myServo.getFrequency());
    myServo.write(90);
    delay(500);
}

void loop() {
    // Sweep 0° -> 180°
    for (int angle = 0; angle <= 180; angle += 5) {
        myServo.write(angle);
        delay(15);
    }
    delay(300);

    // Sweep 180° -> 0°
    for (int angle = 180; angle >= 0; angle -= 5) {
        myServo.write(angle);
        delay(15);
    }
    delay(300);
}
