#include <RoboDrivers.h>

RoboStepDir axis;

void setup() {
  // A4988: STEP, DIR, ENABLE, MS1, MS2, MS3, RESET, SLEEP.
  if (!axis.begin(ROBO_DRIVER_A4988, 2, 3, 4, 5, 6, 7, 8, 9)) {
    for (;;) { }
  }
  if (!axis.setMicrosteps(16)) {
    for (;;) { }
  }
  axis.resetIndexer();
}

void loop() {
  for (int i = 0; i < 3200; ++i) {
    axis.step(true);
    delayMicroseconds(500);
  }
  delay(500);
  for (int i = 0; i < 3200; ++i) {
    axis.step(false);
    delayMicroseconds(500);
  }
  delay(500);
}
