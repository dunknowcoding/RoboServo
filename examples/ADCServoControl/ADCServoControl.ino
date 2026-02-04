/**
 * ADCServoControl.ino - Control servos with potentiometers
 * 
 * Reads 3 potentiometers (ADC) and maps to 3 servo positions.
 * LED brightness shows average position.
 * 
 * Pots: GPIO 34, 35, 36 (input-only pins)
 * Servos: GPIO 13, 14, 15
 * LED: GPIO 2
 */

#include <RoboServo.h>

const int NUM = 3;
const int potPins[] = {34, 35, 36};
const int servoPins[] = {13, 14, 15};
const int LED_PIN = 2;

RoboServo servos[NUM];
float smoothed[NUM] = {0, 0, 0};
int prevAngles[NUM] = {90, 90, 90};

void setup() {
    Serial.begin(115200);
    
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    
    // Set up LED BEFORE servos (timer conflict avoidance)
    analogWrite(LED_PIN, 0);
    
    // Attach servos
    for (int i = 0; i < NUM; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].write(90);
        smoothed[i] = analogRead(potPins[i]);
    }
    
    Serial.println("ADC Servo Control ready");
}

void loop() {
    int totalAngle = 0;
    
    for (int i = 0; i < NUM; i++) {
        // Read and smooth ADC
        int raw = analogRead(potPins[i]);
        smoothed[i] = 0.1 * raw + 0.9 * smoothed[i];
        
        // Map to angle with dead zone
        int angle = map((int)smoothed[i], 0, 4095, 0, 180);
        if (abs(angle - prevAngles[i]) > 2) {
            prevAngles[i] = angle;
            servos[i].write(angle);
        }
        totalAngle += prevAngles[i];
    }
    
    // LED shows average position
    analogWrite(LED_PIN, map(totalAngle / NUM, 0, 180, 0, 255));
    
    delay(10);
}
