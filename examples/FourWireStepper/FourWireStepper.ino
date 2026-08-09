#include <RoboDrivers.h>

RoboFourWireStepper axis;

void setup() {
  if (!axis.begin(ROBO_DRIVER_ULN2003, 8, 9, 10, 11)) {
    for (;;) { }
  }
}

void loop() {
  for (int i = 0; i < 4096; ++i) {
    axis.step(true);
    delayMicroseconds(900);
  }
  axis.release();
  delay(500);
}
