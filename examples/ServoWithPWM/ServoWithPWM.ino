/**
 * ServoWithPWM.ino - Using RoboServo with LED PWM (no timer conflicts)
 * 
 * Key: Use ledcAttach() for LED instead of analogWrite().
 * RoboServo uses 50Hz, analogWrite uses 1000Hz - different timers = safe.
 * 
 * Servo: GPIO 13, LED: GPIO 2
 */

#include <RoboServo.h>

RoboServo servo;
const int SERVO_PIN = 13;
const int LED_PIN = 2;

int brightness = 0;
int fadeDir = 5;

void setup() {
    Serial.begin(115200);
    
    // Attach servo
    servo.attach(SERVO_PIN);
    servo.write(90);
    
    // Use ledcAttach for LED (recommended over analogWrite)
    ledcAttach(LED_PIN, 5000, 8);  // 5kHz, 8-bit
    
    Serial.println("Servo + LED PWM demo");
}

void loop() {
    // Sweep servo
    static int angle = 0;
    static int dir = 1;
    
    angle += dir;
    if (angle >= 180 || angle <= 0) dir = -dir;
    servo.write(angle);
    
    // Fade LED
    brightness += fadeDir;
    if (brightness >= 255 || brightness <= 0) fadeDir = -fadeDir;
    ledcWrite(LED_PIN, brightness);
    
    delay(15);
}

/*
 * Timer conflict tips:
 * - Use ledcAttach() + ledcWrite() instead of analogWrite()
 * - Or call analogWrite() BEFORE servo.attach()
 * - Different frequencies auto-use different timers
 */
