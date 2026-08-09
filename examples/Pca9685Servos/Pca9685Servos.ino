#include <RoboDrivers.h>

RoboPca9685 servos;

void setup() {
  if (!servos.begin(Wire, 0x40)) {
    for (;;) { }
  }
  servos.setFrequency(50);
}

void loop() {
  servos.writeMicroseconds(0, 500);
  delay(500);
  servos.writeMicroseconds(0, 1500);
  delay(500);
  servos.writeMicroseconds(0, 2500);
  delay(500);
}
