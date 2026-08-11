// Basic.ino - minimal RotEv2 usage example
// Runs MOTOR_1 back and forth: 100 rad out, 100 rad back, forever.
//
// NOTE: The rotev library owns core1 for its control loop.
// Do NOT define setup1() or loop1() in your sketch.

#include <rotev.h>
using namespace rotev;

// 100 rad, cruising at 100 rad/s, reaching that speed in 0.5 s.
static Profile fwd;
static Profile back;
static bool going_out = true;

void setup() {
  Serial.begin(115200);
  begin();

  fwd = Profile::fromVelAccel(100.0f, 100.0f, 200.0f);
  back = fwd.scaleDistance(-1.0f);  // same envelope, opposite direction
  fwd.print();

  motorEnable(MOTOR_1);
  motorSetProfile(MOTOR_1, fwd);
  ledColor(0, 0, 255);  // blue = running
}

void loop() {
  ProfileState st = motorProgress(MOTOR_1);
  Serial.printf("t %.2f s  pos %.1f rad  vel %.1f rad/s\n", st.t, st.pos, st.vel);

  // The profile is executed by the control loop on core1, so all this loop
  // has to do is hand over the next leg once the current one lands.
  if (st.done) {
    going_out = !going_out;
    motorSetProfile(MOTOR_1, going_out ? fwd : back);
  }

  delay(100);
}
