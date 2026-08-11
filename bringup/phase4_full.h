#pragma once
#include <Arduino.h>
#include <rotev.h>
#include <rotev_internal.h>
using namespace rotev;

// Back and forth on both motors. One CYCLE is a leg out of TARGET_REV
// rotations plus the leg back, both on the same envelope -- the return is
// literally the forward profile mirrored (scaleDistance(-1)).
//
// The profile owns position: the ISR runs it off its own tick counter, so a
// leg ends exactly on its distance no matter how badly core0 jitters, and
// nothing here has to track where the axis is.
//
// With nothing plugged into a motor's connector its current loop simply
// integrates up against an open circuit: no current flows, the duty saturates
// harmlessly, and the online Lq estimator skips itself.
static constexpr float TARGET_REV = 25.0f;
static constexpr int CYCLES = 5;  // forward+back pairs
static constexpr float VMAX = 1600.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
// The ramp is at constant acceleration, so ta = vpk/a: this is the
// acceleration that gets to VMAX in RAMP_S.
static constexpr float RAMP_S = 0.25f;
static constexpr float AMAX = VMAX / RAMP_S;

static Profile fwd, back;
static int legs[2] = {0, 0};  // completed legs, 2 per cycle
static uint32_t last_print_us = 0;

// Issue the next leg, or shut the axis down once the cycles are done.
static void nextLeg(Motor m) {
  int& n = legs[m];
  if (n >= 2 * CYCLES) return;
  ++n;
  if (n >= 2 * CYCLES) {
    motorEnable(m, false);  // don't hold current forever
    Serial.print("# motor ");
    Serial.print((int)m + 1);
    Serial.println(" done");
    return;
  }
  motorSetProfile(m, (n % 2 == 0) ? fwd : back);
}

void setup() {
  Serial.begin(115200);
  begin();
  fwd = Profile::fromVelAccel(TARGET_REV * 2.0f * PI, VMAX, AMAX);
  back = fwd.scaleDistance(-1.0f);
  fwd.print();
  motorSetProfile(MOTOR_1, fwd);
  motorSetProfile(MOTOR_2, fwd);
  motorEnable(MOTOR_1);
  motorEnable(MOTOR_2);
  last_print_us = micros();
}

void loop() {
  if (motorProgress(MOTOR_1).done) nextLeg(MOTOR_1);
  if (motorProgress(MOTOR_2).done) nextLeg(MOTOR_2);

  uint32_t now = micros();
  if (now - last_print_us < 25000) return;
  last_print_us = now;

  ProfileState s1 = motorProgress(MOTOR_1);
  ProfileState s2 = motorProgress(MOTOR_2);
  Serial.print(">vbus:");
  Serial.print(busVoltage(), 2);
  Serial.print(",leg1:");
  Serial.print(legs[MOTOR_1]);
  Serial.print(",rev1:");
  Serial.print(s1.pos / (2.0f * PI), 2);
  Serial.print(",rpm1:");
  Serial.print(s1.vel * 60.0f / (2.0f * PI), 0);
  Serial.print(",id1:");
  Serial.print(motorCurrentD(MOTOR_1), 3);
  Serial.print(",iq1:");
  Serial.print(motorCurrentQ(MOTOR_1), 3);
  Serial.print(",ud1:");
  Serial.print(motorVoltageD(MOTOR_1), 2);
  Serial.print(",leg2:");
  Serial.print(legs[MOTOR_2]);
  Serial.print(",rev2:");
  Serial.print(s2.pos / (2.0f * PI), 2);
  Serial.print(",rpm2:");
  Serial.print(s2.vel * 60.0f / (2.0f * PI), 0);
  Serial.print(",id2:");
  Serial.print(motorCurrentD(MOTOR_2), 3);
  Serial.print(",iq2:");
  Serial.print(motorCurrentQ(MOTOR_2), 3);
  Serial.print(",ud2:");
  Serial.println(motorVoltageD(MOTOR_2), 2);
}
