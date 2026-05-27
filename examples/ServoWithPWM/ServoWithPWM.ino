/**
 * ServoWithPWM.ino - Using RoboServo with LED PWM (no timer conflicts)
 * 
 * ESP32: Use ledcAttach() for LED instead of analogWrite().
 * ESP8266: Call analogWrite() on the LED pin BEFORE servo.attach().
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
    
#if defined(ESP8266)
    // ESP8266: initialize LED PWM before servo attach (shared timer)
    analogWrite(LED_PIN, 0);
#else
    // ESP32: use ledcAttach for LED (recommended over analogWrite)
    ledcAttach(LED_PIN, 5000, 8);  // 5kHz, 8-bit
#endif

    // Attach servo
    servo.attach(SERVO_PIN);
    servo.write(90);
    
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
#if defined(ESP8266)
    analogWrite(LED_PIN, brightness);
#else
    ledcWrite(LED_PIN, brightness);
#endif
    
    delay(15);
}

/*
 * Timer conflict tips:
 * ESP32:
 *   - Use ledcAttach() + ledcWrite() instead of analogWrite()
 *   - Or call analogWrite() BEFORE servo.attach()
 *   - Different frequencies auto-use different timers
 * ESP8266:
 *   - All PWM shares one frequency — call analogWrite() before servo.attach()
 */
