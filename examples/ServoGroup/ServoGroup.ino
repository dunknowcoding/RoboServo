/**
 * ServoGroup.ino - Coordinated control of multiple servos
 * 
 * Uses RoboServoGroup to move servos together or in patterns.
 * Servos: GPIO 13, 14, 15
 */

#include <RoboServo.h>

RoboServoGroup group;
const int pins[] = {13, 14, 15};
const int NUM = 3;

void setup() {
    Serial.begin(115200);
    
    // Add servos to group
    for (int i = 0; i < NUM; i++) {
        group.addServo(pins[i]);
    }
    group.writeAll(90);
    
    Serial.printf("Group created with %d servos\n", group.count());
}

void loop() {
    // All servos to same angle
    Serial.println("writeAll demo");
    int angles[] = {0, 90, 180, 90};
    for (int i = 0; i < 4; i++) {
        group.writeAll(angles[i]);
        delay(600);
    }
    
    // Each servo to different angle
    Serial.println("writeMultiple demo");
    int pattern1[] = {0, 90, 180};
    int pattern2[] = {180, 90, 0};
    group.writeMultiple(pattern1, NUM);
    delay(800);
    group.writeMultiple(pattern2, NUM);
    delay(800);
    group.writeAll(90);
    
    // Wave effect
    Serial.println("Wave demo");
    for (int angle = 0; angle <= 180; angle += 3) {
        int a[3];
        for (int i = 0; i < NUM; i++) {
            int off = (i * 40);
            a[i] = constrain(angle + off, 0, 180);
        }
        group.writeMultiple(a, NUM);
        delay(15);
    }
    group.writeAll(90);
    
    delay(2000);
}
