/**
 * Servo360.ino - Continuous rotation servo control
 * 
 * 360° servos are speed-controlled, not position-controlled.
 * - setSpeed(-100 to +100): Intuitive speed control
 * - write(0)=full CW, write(90)=stop, write(180)=full CCW
 * 
 * Servo: GPIO 13
 */

#include <RoboServo.h>

RoboServo servo;
const int PIN = 13;

void setup() {
    Serial.begin(115200);
    
    servo.attach(PIN, 500, 2500, SERVO_TYPE_360);
    servo.stop();
    
    Serial.println("360° servo ready");
    delay(1000);
}

void loop() {
    // Using setSpeed(): -100 (reverse) to +100 (forward)
    Serial.println("setSpeed demo:");
    
    servo.setSpeed(100);   // Full forward
    Serial.println("  +100 (full forward)");
    delay(1500);
    
    servo.setSpeed(50);    // Half forward
    Serial.println("  +50 (half forward)");
    delay(1500);
    
    servo.stop();
    Serial.println("  stop");
    delay(500);
    
    servo.setSpeed(-50);   // Half reverse
    Serial.println("  -50 (half reverse)");
    delay(1500);
    
    servo.setSpeed(-100);  // Full reverse
    Serial.println("  -100 (full reverse)");
    delay(1500);
    
    servo.stop();
    delay(1000);
    
    // Smooth acceleration
    Serial.println("Smooth ramp up/down:");
    for (int spd = 0; spd <= 100; spd += 10) {
        servo.setSpeed(spd);
        delay(100);
    }
    for (int spd = 100; spd >= -100; spd -= 10) {
        servo.setSpeed(spd);
        delay(100);
    }
    for (int spd = -100; spd <= 0; spd += 10) {
        servo.setSpeed(spd);
        delay(100);
    }
    
    delay(2000);
}
