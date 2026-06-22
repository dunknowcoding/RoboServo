#include <RoboServo.h>

RoboServo myServo;

void setup() {
    if (myServo.attach(13) == ROBOSERVO_INVALID_SERVO) {
        while (1) {}
    }
    myServo.write(90);
}

void loop() {
}
