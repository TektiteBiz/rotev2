// Basic.ino — minimal RotEv2 FOC usage example
// Spins MOTOR_1 at 0.3 A in an open-loop position sweep.
//
// NOTE: The rotev library owns core1 for its FOC loop.
// Do NOT define setup1() or loop1() in your sketch.

#include <rotev.h>
using namespace rotev;

void setup() {
  begin();
  motorEnable(MOTOR_1);
  ledColor(0, 0, 255);  // blue = running
}

void loop() {
  static float theta = 0;
  theta += 0.01f;
  motorWrite(theta, 0.3f, MOTOR_1);  // advance position, 0.3 A
  delay(1);
}
