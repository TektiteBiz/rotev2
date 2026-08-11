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

static constexpr float TARGET_REV = 500.0f;
static constexpr float VMAX = 1600.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
static constexpr float AMAX = VMAX / 1.0f;  // reach VMAX in 0.25 s
static constexpr float AMPS = 0.8f;

struct Axis {
  Motor m;
  bool on;
  double theta;  // full-precision profile position, core0 only
  // The same angle wrapped to one revolution -- this is what gets commanded.
  // Wrapped incrementally rather than with fmod() on the double every
  // iteration: this core has no double-precision FPU, so that would put a
  // software call in the hot path. Both integrate the same vel*dt, so it
  // stays equal to theta modulo 2*PI to well under a milliradian across the
  // whole 100-revolution run.
  float theta_cmd;
  float vel;
};

static Axis ax[] = {
    {MOTOR_1, true, 0.0, 0.0f, 0.0f},
    {MOTOR_2, true, 0.0, 0.0f, 0.0f},
};
static constexpr int N_AX = sizeof(ax) / sizeof(ax[0]);

static uint32_t last_us = 0, last_print_us = 0;
static uint32_t loop_count = 0, max_dt_us = 0;

static void stepAxis(Axis& a, float dt) {
  if (!a.on) return;
  const double target = (double)TARGET_REV * 2.0 * PI;
  double remaining = target - a.theta;
  // Report the profile's INTENDED acceleration rather than differencing
  // velocity: at a ~300 kHz loop rate dt is quantized to whole microseconds,
  // so a numerical derivative would be mostly noise. Each branch below knows
  // its own second derivative analytically.
  float a_cmd = 0.0f;
  if (remaining <= 0.0) {
    // Land exactly on target rather than a fraction past it. The decel
    // envelope drives vel to zero as remaining does, so the overshoot is
    // second-order small, but pinning it here means the commanded angle the
    // ISR converges to is the profile's exact endpoint. The ISR reaches it
    // asymptotically: tau = CTRL_DT / PROF_TRACK ~ 17 ms, so ~0.2 s after the
    // profile finishes.
    a.theta = target;
    a.vel = 0.0f;
  } else {
    float vstop = sqrtf(2.0f * AMAX * (float)remaining);  // decel envelope
    float vcmd;
    if (a.vel < VMAX) {
      vcmd = a.vel + AMAX * dt;
      a_cmd = AMAX;
    } else {
      vcmd = VMAX;
      a_cmd = 0.0f;
    }
    if (vcmd > vstop) {
      vcmd = vstop;
      a_cmd = -AMAX;  // riding the envelope down at constant deceleration
    }
    a.vel = vcmd;
    a.theta += (double)a.vel * dt;
    a.theta_cmd += a.vel * dt;
    if (a.theta_cmd >= 2.0f * PI) a.theta_cmd -= 2.0f * PI;
  }
  motorWriteProfile(a.theta_cmd, a.vel, a_cmd, AMPS, a.m);
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
  uint32_t dt_us = now - last_us;
  float dt = dt_us * 1e-6f;
  last_us = now;
  ++loop_count;
  if (dt_us > max_dt_us) max_dt_us = dt_us;

  for (int i = 0; i < N_AX; ++i) stepAxis(ax[i], dt);

  uint32_t print_dt_us = now - last_print_us;
  if (print_dt_us >= 25000) {
    last_print_us = now;
    Serial.print(">loopHz:");
    Serial.print(loop_count * 1e6f / print_dt_us, 0);
    Serial.print(",maxDtUs:");
    Serial.print(max_dt_us);
    Serial.print(",vbus:");
    Serial.print(busVoltage(), 2);
    for (int i = 0; i < N_AX; ++i) {
      if (!ax[i].on) continue;
      int n = i + 1;
      Serial.print(",rev");
      Serial.print(n);
      Serial.print(':');
      Serial.print((float)(ax[i].theta / (2.0 * PI)), 2);
      Serial.print(",rpm");
      Serial.print(n);
      Serial.print(':');
      Serial.print(ax[i].vel * 60.0f / (2.0f * PI), 0);
      Serial.print(",iq");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorCurrentQ(ax[i].m), 3);
      Serial.print(",id");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorCurrentD(ax[i].m), 3);
      Serial.print(",ud");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorVoltageD(ax[i].m), 2);
      Serial.print(",derate");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorDerate(ax[i].m), 3);
    }
    Serial.println();
    loop_count = 0;
    max_dt_us = 0;
  }
  // No delay: the setpoint is consumed by the 12 kHz-per-motor control ISR.
}
