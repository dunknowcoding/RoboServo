/**
 * MultipleServos.ino - Control multiple servos independently
 * 
 * Servos: GPIO 13, 14, 15, 16
 */

#include <RoboServo.h>

RoboServo servos[4];
const int pins[] = {13, 14, 15, 16};

void setup() {
    Serial.begin(115200);
    
    // Attach all servos
    for (int i = 0; i < 4; i++) {
        servos[i].attach(pins[i]);
        servos[i].write(90);
    }
    Serial.println("4 servos attached");
}

void loop() {
    wavePattern();
    delay(500);
    
    synchronizedSweep();
    delay(500);
    
    opposingMotion();
    delay(500);
}

// Wave: each servo follows the previous
void wavePattern() {
    Serial.println("Wave pattern");
    for (int angle = 0; angle <= 180; angle += 10) {
        for (int i = 0; i < 4; i++) {
            servos[i].write(angle);
            delay(40);
        }
    }
    for (int angle = 180; angle >= 0; angle -= 10) {
        for (int i = 0; i < 4; i++) {
            servos[i].write(angle);
            delay(40);
        }
    }
}

// All servos move together
void synchronizedSweep() {
    Serial.println("Synchronized sweep");
    for (int angle = 0; angle <= 180; angle += 5) {
        for (int i = 0; i < 4; i++) servos[i].write(angle);
        delay(20);
    }
    for (int angle = 180; angle >= 0; angle -= 5) {
        for (int i = 0; i < 4; i++) servos[i].write(angle);
        delay(20);
    }
}

// Alternating servos move in opposite directions
void opposingMotion() {
    Serial.println("Opposing motion");
    for (int angle = 0; angle <= 180; angle += 5) {
        servos[0].write(angle);
        servos[2].write(angle);
        servos[1].write(180 - angle);
        servos[3].write(180 - angle);
        delay(20);
    }
    for (int angle = 180; angle >= 0; angle -= 5) {
        servos[0].write(angle);
        servos[2].write(angle);
        servos[1].write(180 - angle);
        servos[3].write(180 - angle);
        delay(20);
    }
}
