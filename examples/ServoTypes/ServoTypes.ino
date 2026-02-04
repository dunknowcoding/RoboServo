/**
 * ServoTypes.ino - Control servos with different rotation ranges
 * 
 * Demonstrates 3 ways to set servo range:
 *   1. servo.attach(pin, minUs, maxUs, SERVO_TYPE_180)  // Predefined type
 *   2. servo.attach(pin, minUs, maxUs, 90)             // Custom angle
 *   3. servo.setMaxAngle(90)                           // After attach
 */

#include <RoboServo.h>

// Servo objects
RoboServo servo180;
RoboServo servo270;
RoboServo servo90;

// Pulse width settings (adjust for your servos)
int minUs1 = 500,  maxUs1 = 2500;  // 180° servo
int minUs2 = 500,  maxUs2 = 2500;  // 270° servo
int minUs3 = 1000, maxUs3 = 2000;  // 90° servo (narrower range)

// Pin assignments
int pin1 = 13;  // 180° servo
int pin2 = 14;  // 270° servo
int pin3 = 15;  // 90° servo

void setup() {
    Serial.begin(115200);
    
    // Attach servos with different max angles
    servo180.attach(pin1, minUs1, maxUs1, SERVO_TYPE_180);
    servo270.attach(pin2, minUs2, maxUs2, SERVO_TYPE_270);
    servo90.attach(pin3, minUs3, maxUs3, 90);  // Custom 90° range
    
    Serial.println("Servo Ranges Demo");
    Serial.printf("Servo 1: %d° max\n", servo180.getMaxAngle());
    Serial.printf("Servo 2: %d° max\n", servo270.getMaxAngle());
    Serial.printf("Servo 3: %d° max\n", servo90.getMaxAngle());
}

void loop() {
    // Sweep each servo through its full range
    sweepServo(servo180, "180°", 0, 180, 30);
    sweepServo(servo270, "270°", 0, 270, 45);
    sweepServo(servo90,  "90°",  0, 90,  15);
    
    delay(2000);
}

// Sweep servo from start to end angle, then back
void sweepServo(RoboServo &servo, const char* name, int start, int end, int step) {
    Serial.printf("\n%s servo sweep:\n", name);
    
    for (int pos = start; pos <= end; pos += step) {
        servo.write(pos);
        Serial.printf("  %d°\n", pos);
        delay(400);
    }
    for (int pos = end; pos >= start; pos -= step) {
        servo.write(pos);
        delay(250);
    }
}
