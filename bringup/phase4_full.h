#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Trapezoidal position profile to TARGET_REV rotations, run on both motors.
//
// Commands POSITION plus a velocity feedforward (motorWriteProfile). Position
// is authoritative -- the ISR applies the angle exactly as given on every
// update -- so this profile, not an ISR-side integral, decides where each
// axis ends up and no drift can accumulate. The velocity term only
// interpolates across the gap since the last update, which keeps the field
// advancing smoothly if core0 stalls (the USB CDC write below can block ~1 ms).
//
// Plain position (motorWrite) is not enough on its own: it leaves the field
// frozen for the whole of any core0 stall and then jumps. That is what made
// this phase sound terrible -- with the old delayMicroseconds(500) the
// setpoint advanced in ~60 electrical degree steps at 400 RPM -- and it also
// handed a staircased angle to the speed estimator, which now feeds the
// voltage derate, the decoupling and the delay compensation.
//
// Each axis keeps its own profile state, so the two can be given different
// targets later by editing the per-axis fields rather than the loop. Both are
// driven by the same 24 kHz ISR, which alternates motors and therefore
// services each at 12 kHz -- adding the second motor does not change the rate
// either one already had.
//
// theta is a double because the target is 628 rad, where float resolution
// (6e-5) is close to one iteration's increment. It is wrapped to one
// mechanical revolution before being commanded: POLE_PAIRS is an integer, so
// that leaves the electrical angle untouched while keeping the float handed
// to the ISR small and precise.
//
// With nothing plugged into a motor's connector its current loop simply
// integrates up against an open circuit: no current flows, the duty saturates
// harmlessly, and the online Lq estimator skips itself (it is gated on iq
// being non-trivial). Set the axis's `on` field false to leave it idle.

static constexpr float TARGET_REV = 100.0f;
static constexpr float VMAX = 400.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
static constexpr float AMAX = VMAX / 0.25f;                // reach VMAX in 0.25 s
static constexpr float AMPS = 0.8f;

struct Axis {
  Motor  m;
  bool   on;
  double theta;   // full-precision profile position, core0 only
  float  vel;
};

static Axis ax[] = {
    {MOTOR_1, true, 0.0, 0.0f},
    {MOTOR_2, true, 0.0, 0.0f},
};
static constexpr int N_AX = sizeof(ax) / sizeof(ax[0]);

static uint32_t last_us = 0, last_print_us = 0;

static void stepAxis(Axis& a, float dt) {
  if (!a.on) return;
  const double target = (double)TARGET_REV * 2.0 * PI;
  double remaining = target - a.theta;
  if (remaining <= 0.0) {
    a.vel = 0.0f;
  } else {
    float vstop = sqrtf(2.0f * AMAX * (float)remaining);  // decel envelope
    float vcmd = (a.vel < VMAX) ? a.vel + AMAX * dt : VMAX;
    if (vcmd > vstop) vcmd = vstop;
    a.vel = vcmd;
    a.theta += (double)a.vel * dt;
  }
  double thw = fmod(a.theta, 2.0 * PI);
  if (thw < 0.0) thw += 2.0 * PI;
  motorWriteProfile((float)thw, a.vel, AMPS, a.m);
}

void setup() {
  Serial.begin(115200);
  begin();
  for (int i = 0; i < N_AX; ++i)
    if (ax[i].on) motorEnable(ax[i].m);
  last_us = last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;

  for (int i = 0; i < N_AX; ++i) stepAxis(ax[i], dt);

  if (now - last_print_us >= 25000) {
    last_print_us = now;
    Serial.print(">vbus:");
    Serial.print(busVoltage(), 2);
    for (int i = 0; i < N_AX; ++i) {
      if (!ax[i].on) continue;
      int n = i + 1;
      Serial.print(",rev");    Serial.print(n); Serial.print(':');
      Serial.print((float)(ax[i].theta / (2.0 * PI)), 2);
      Serial.print(",rpm");    Serial.print(n); Serial.print(':');
      Serial.print(ax[i].vel * 60.0f / (2.0f * PI), 0);
      Serial.print(",iq");     Serial.print(n); Serial.print(':');
      Serial.print(motorCurrentQ(ax[i].m), 3);
      Serial.print(",id");     Serial.print(n); Serial.print(':');
      Serial.print(motorCurrentD(ax[i].m), 3);
      Serial.print(",ud");     Serial.print(n); Serial.print(':');
      Serial.print(motorVoltageD(ax[i].m), 2);
      Serial.print(",derate"); Serial.print(n); Serial.print(':');
      Serial.print(motorDerate(ax[i].m), 3);
    }
    Serial.println();
  }
  // No delay: the setpoint is consumed by the 12 kHz-per-motor control ISR.
}
