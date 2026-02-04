/**
 * CustomPulseWidth.ino - Calibrate servo pulse width range
 * 
 * Serial commands:
 *   a/A = decrease/increase min pulse by 10us
 *   z/Z = decrease/increase max pulse by 10us
 *   0/9/1 = move to 0°/90°/180°
 *   s = sweep test, p = print settings
 */

#include <RoboServo.h>

RoboServo servo;
const int PIN = 13;

int minPulse = 500;
int maxPulse = 2500;

void setup() {
    Serial.begin(115200);
    servo.attach(PIN, minPulse, maxPulse);
    servo.write(90);
    
    Serial.println("Pulse calibration tool");
    Serial.printf("Current: %d-%d us\n", minPulse, maxPulse);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        
        switch (c) {
            case 'a': minPulse -= 10; updatePulse(); break;
            case 'A': minPulse += 10; updatePulse(); break;
            case 'z': maxPulse -= 10; updatePulse(); break;
            case 'Z': maxPulse += 10; updatePulse(); break;
            case '0': servo.write(0);   Serial.println("-> 0°");   break;
            case '9': servo.write(90);  Serial.println("-> 90°");  break;
            case '1': servo.write(180); Serial.println("-> 180°"); break;
            case 's': sweepTest(); break;
            case 'p': 
                Serial.printf("Min: %dus, Max: %dus, Pos: %d°\n", 
                    servo.getMinPulse(), servo.getMaxPulse(), servo.read());
                break;
        }
    }
}

void updatePulse() {
    minPulse = constrain(minPulse, 200, 1500);
    maxPulse = constrain(maxPulse, 1500, 3000);
    servo.setPulseLimits(minPulse, maxPulse);
    Serial.printf("Pulse: %d-%d us\n", minPulse, maxPulse);
}

void sweepTest() {
    Serial.println("Sweep test...");
    for (int a = 0; a <= 180; a += 5) { servo.write(a); delay(30); }
    for (int a = 180; a >= 0; a -= 5) { servo.write(a); delay(30); }
    servo.write(90);
    Serial.println("Done");
}
