#pragma once
#include <Arduino.h>
#include <rotev.h>
#include <rotev_internal.h>
using namespace rotev;

// Closed-loop FOC: one long profile on both motors, then just watch it run.
//
// controlStep() has no position sensor -- it takes the COMMANDED theta as a
// stand-in for the true rotor position and uses it to park-transform the
// measured current for feedback. At power-up the real rotor angle is
// arbitrary, so that assumption starts out false. The profile's own ramp from
// zero is what fixes it: for the first moments the commanded field is barely
// moving while already carrying full current, which is exactly an alignment
// hold, and the rotor is captured long before the speed is high enough to
// outrun it. A separate explicit align stage was redundant with that.
//
// The 24 kHz ISR alternates motors and so services each at 12 kHz; motor 2
// was always being stepped, just with a zero setpoint, so enabling it costs
// motor 1 nothing. With nothing plugged into a connector that axis simply
// integrates up against an open circuit -- no current, saturated duty, and
// the online Lq estimator skips itself since it is gated on |iq|.
static constexpr float DISTANCE_RAD = 25000.0f;             // ~2.4 min of run
static constexpr float VMAX = 1700.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
// The ramp is at constant acceleration, so ta = vpk/a: this is the
// acceleration that gets to VMAX in RAMP_S.
static constexpr float RAMP_S = 2.0f;
static constexpr float AMAX = VMAX / RAMP_S;

static uint32_t last_print_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  Profile p = Profile::fromVelAccel(DISTANCE_RAD, VMAX, AMAX);
  p.print();
  motorSetProfile(MOTOR_1, p);
  motorSetProfile(MOTOR_2, p);
  motorEnable(MOTOR_1);
  motorEnable(MOTOR_2);
  last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  if (now - last_print_us < 25000) return;
  last_print_us = now;

  ProfileState s1 = motorProgress(MOTOR_1);
  ProfileState s2 = motorProgress(MOTOR_2);
  // Electrical degrees per commanded-field step, set by the control ISR tick
  // (PWM_HZ/2 = 12 kHz). Compare against 90 deg (a quarter cycle); below ~20
  // steps per electrical cycle the current stops being able to look like a
  // sine no matter how much voltage headroom there is.
  float deg_e_step =
      s1.vel * (180.0f / PI) * (float)POLE_PAIRS / (PWM_HZ / 2.0f);

  Serial.print(">t:");
  Serial.print(s1.t, 2);
  Serial.print(",rpm1:");
  Serial.print(s1.vel * 60.0f / (2.0f * PI), 0);
  Serial.print(",rpm2:");
  Serial.print(s2.vel * 60.0f / (2.0f * PI), 0);
  Serial.print(",degEStep:");
  Serial.print(deg_e_step, 1);
  Serial.print(",vbus:");
  Serial.print(busVoltage(), 2);
  Serial.print(",id1:");
  Serial.print(motorCurrentD(MOTOR_1), 3);
  Serial.print(",iq1:");
  Serial.print(motorCurrentQ(MOTOR_1), 3);
  Serial.print(",ud1:");
  Serial.print(motorVoltageD(MOTOR_1), 2);
  Serial.print(",uq1:");
  Serial.print(motorVoltageQ(MOTOR_1), 2);
  Serial.print(",id2:");
  Serial.print(motorCurrentD(MOTOR_2), 3);
  Serial.print(",iq2:");
  Serial.print(motorCurrentQ(MOTOR_2), 3);
  Serial.print(",ud2:");
  Serial.print(motorVoltageD(MOTOR_2), 2);
  Serial.print(",uq2:");
  Serial.println(motorVoltageQ(MOTOR_2), 2);
}
