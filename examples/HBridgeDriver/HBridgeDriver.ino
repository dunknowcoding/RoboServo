#include <RoboDrivers.h>

// TB6612 channel A: AIN1, AIN2, PWMA, STBY.
RoboHBridge motor;

void setup() {
  if (!motor.beginIn1In2(ROBO_DRIVER_TB6612, 4, 5, 6, 7)) {
    for (;;) { }
  }
}

void loop() {
  motor.drive(60);
  delay(1000);
  motor.brake();
  delay(250);
  motor.drive(-60);
  delay(1000);
  motor.coast();
  delay(500);
}
