/**
 * BasicServo.ino - Single servo control example
 * 
 * Wiring: Servo signal -> GPIO 13, VCC -> 5V, GND -> GND
 */

#include <RoboServo.h>

RoboServo myServo;
const int SERVO_PIN = 13;

void setup() {
    Serial.begin(115200);
    
    if (myServo.attach(SERVO_PIN) == ROBOSERVO_INVALID_SERVO) {
        Serial.println("Failed to attach servo!");
        while (1) delay(1000);
    }
    
    Serial.println("Servo attached. Starting sweep...");
    myServo.write(90);  // Start at center
    delay(500);
}

void loop() {
    // Sweep 0° -> 180°
    for (int angle = 0; angle <= 180; angle += 5) {
        myServo.write(angle);
        delay(30);
    }
    delay(500);
    
    // Sweep 180° -> 0°
    for (int angle = 180; angle >= 0; angle -= 5) {
        myServo.write(angle);
        delay(30);
    }
    delay(500);
}
