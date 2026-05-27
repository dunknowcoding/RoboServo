/**
 * MotorGroup.ino - Coordinated control of multiple motors
 *
 * Uses RoboMotorGroup to drive motors together or in patterns.
 * Motors: GPIO 14, 15
 */

#include <RoboMotor.h>

RoboMotorGroup group;
const int pins[] = {14, 15};
const int NUM = 2;

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM; i++) {
        group.addMotor(pins[i]);
    }
    group.stopAll();

    Serial.printf("Motor group created with %d motors\n", group.count());
}

void loop() {
    // All motors to same duty
    Serial.println("writeAll demo");
    int duties[] = {0, 25, 50, 75, 100, 50, 0};
    for (int i = 0; i < 7; i++) {
        group.writeAll(duties[i]);
        delay(600);
    }

    // Each motor to different duty
    Serial.println("writeMultiple demo");
    int pattern1[] = {20, 80};
    int pattern2[] = {80, 20};
    group.writeMultiple(pattern1, NUM);
    delay(800);
    group.writeMultiple(pattern2, NUM);
    delay(800);
    group.stopAll();

    delay(2000);
}
