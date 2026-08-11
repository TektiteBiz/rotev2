// Basic.ino - minimal RotEv2 usage example
// Runs both motors back and forth: 100 rad out, 100 rad back, with MOTOR_2
// always travelling opposite to MOTOR_1, chirping on the buzzer each time a
// leg lands. The GO button starts it, the STOP button stops it; the LED is
// blue while running and red while stopped.
//
// NOTE: The rotev library owns core1 for its control loop.
// Do NOT define setup1() or loop1() in your sketch.

#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// 100 rad, cruising at 100 rad/s, reaching that speed in 0.5 s.
static Profile fwd;
static Profile back;
static bool going_out = true;
static bool running = false;

// Rising three-note chirp. buzzerOn() retunes while already sounding, so
// there is no need to switch it off between notes. Frequencies are clamped
// to 1-4 kHz by the library.
static void playDone() {
  const uint16_t notes[] = {1500, 2000, 3000};
  for (uint16_t f : notes) {
    buzzerOn(f);
    delay(70);
  }
  buzzerOff();
}

// Hand the current leg to both motors. The two profiles are mirror images of
// each other, so the motors turn in opposite directions over the same
// envelope and land at the same moment.
static void startLeg() {
  motorSetProfile(MOTOR_1, going_out ? fwd : back);
  motorSetProfile(MOTOR_2, going_out ? back : fwd);
}

void setup() {
  Serial.begin(115200);
  begin();

  fwd = Profile::fromVelAccel(100.0f, 100.0f, 200.0f);
  back = fwd.scaleDistance(-1.0f);  // same envelope, opposite direction
  fwd.print();

  ledColor(255, 0, 0);  // red = stopped
}

void loop() {
  // The buttons are active-high; act on the press edge so that holding one
  // down does not keep restarting the current leg.
  static bool go_prev = false, stop_prev = false;
  bool go_now = buttonPressed(BTN_GO);
  bool stop_now = buttonPressed(BTN_STOP);
  bool go_edge = go_now && !go_prev;
  bool stop_edge = stop_now && !stop_prev;
  go_prev = go_now;
  stop_prev = stop_now;

  // STOP wins if both are pressed in the same pass.
  if (stop_edge && running) {
    running = false;
    motorEnable(MOTOR_1, false);
    motorEnable(MOTOR_2, false);
    ledColor(255, 0, 0);  // red = stopped
  } else if (go_edge && !running) {
    running = true;
    motorEnable(MOTOR_1);
    motorEnable(MOTOR_2);
    // motorSetProfile() starts from the current position, so this restarts
    // the leg that was interrupted rather than resuming partway through it.
    startLeg();
    ledColor(0, 0, 255);  // blue = running
  }

  if (!running) {
    delay(10);
    return;
  }

  ProfileState s1 = motorProgress(MOTOR_1);
  ProfileState s2 = motorProgress(MOTOR_2);
  Serial.print("vbus ");
  Serial.print(busVoltage(), 2);
  Serial.print(" V  t ");
  Serial.print(s1.t, 2);
  Serial.print(" s  m1 ");
  Serial.print(s1.pos, 1);
  Serial.print(" rad ");
  Serial.print(s1.vel, 1);
  Serial.print(" rad/s  m2 ");
  Serial.print(s2.pos, 1);
  Serial.print(" rad ");
  Serial.print(s2.vel, 1);
  Serial.println(" rad/s");

  // The profiles are executed by the control loop on core1, so all this loop
  // has to do is hand over the next leg once the current one lands. Both
  // motors run the same envelope, so MOTOR_1 landing means both have landed.
  if (s1.done) {
    playDone();
    going_out = !going_out;
    startLeg();
  }

  delay(100);
}
