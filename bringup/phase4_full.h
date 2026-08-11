#pragma once
#include <Arduino.h>
#include <math.h>
#include <rotev.h>
using namespace rotev;

// Trapezoidal position profile run back and forth on both motors.
//
// Each CYCLE is one leg forward of TARGET_REV rotations followed by one leg
// back to the start, both using the same accel/cruise/decel envelope. Set
// CYCLES <= 0 to repeat forever.
//
// Commands POSITION plus velocity and acceleration feedforward
// (motorWriteProfile). Position is authoritative, so this profile -- not an
// ISR-side integral -- decides where each axis ends up and no drift can
// accumulate. The ISR still generates the angle on its own hardware tick and
// only tracks the commanded position slowly (PROF_TRACK), which is what keeps
// the field as smooth as velocity mode while staying exact.
//
// theta is a double because the target is 628 rad, where float resolution
// (6e-5) is close to one iteration's increment. theta_cmd is the same angle
// wrapped to one revolution -- wrapped incrementally rather than with fmod()
// on the double, since this core has no double FPU and that would put a
// software call in the hot path. POLE_PAIRS is an integer, so wrapping at
// 2*PI mechanical leaves the electrical angle untouched.
//
// With nothing plugged into a motor's connector its current loop simply
// integrates up against an open circuit: no current flows, the duty saturates
// harmlessly, and the online Lq estimator skips itself. Set an axis's `on`
// field false to leave it idle.

static constexpr float TARGET_REV = 25.0f;
static constexpr int CYCLES = 5;  // forward+back pairs; <= 0 = forever
static constexpr float VMAX = 1600.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
static constexpr float AMAX = VMAX / 0.25f;  // reach VMAX in 0.25 s
static constexpr float AMPS = 0.8f;

// Pause at each turnaround. The ISR converges on a commanded position
// exponentially with tau = CTRL_DT / PROF_TRACK ~ 17 ms, so reversing the
// instant the profile hits its target would command the new direction while
// the field was still settling into the old one. ~5 tau lets it arrive first.
// Set to 0 to reverse immediately.
static constexpr float DWELL_S = 0.01f;

// Treat a leg as complete inside this much of its target. The decel envelope
// drives velocity to zero as the remaining distance does, so without a
// tolerance the last fraction of a radian takes arbitrarily long.
static constexpr double POS_TOL = 1e-3;

struct Axis {
  Motor m;
  bool on;
  double theta;     // absolute profile position, core0 only
  double target;    // where the current leg is headed
  float theta_cmd;  // theta wrapped to one revolution; what gets commanded
  float vel;        // SIGNED -- negative on the return legs
  int leg;          // completed legs; 2 per cycle
  float dwell;      // seconds left of the turnaround pause
};

static Axis ax[] = {
    {MOTOR_1, true, 0.0, 0.0, 0.0f, 0.0f, 0, 0.0f},
    {MOTOR_2, true, 0.0, 0.0, 0.0f, 0.0f, 0, 0.0f},
};
static constexpr int N_AX = sizeof(ax) / sizeof(ax[0]);

static const double FWD = (double)TARGET_REV * 2.0 * PI;

static uint32_t last_us = 0, last_print_us = 0;
static uint32_t loop_count = 0, max_dt_us = 0;
static bool all_done = false;

static inline bool legsRemain(const Axis& a) {
  return CYCLES <= 0 || a.leg < 2 * CYCLES;
}

static void stepAxis(Axis& a, float dt) {
  if (!a.on) return;

  if (!legsRemain(a)) {
    a.vel = 0.0f;
    motorWriteProfile(a.theta_cmd, 0.0f, 0.0f, AMPS, a.m);
    return;
  }
  if (a.dwell > 0.0f) {
    a.dwell -= dt;
    a.vel = 0.0f;
    motorWriteProfile(a.theta_cmd, 0.0f, 0.0f, AMPS, a.m);
    return;
  }

  double remaining = a.target - a.theta;
  if (fabs(remaining) <= POS_TOL) {  // leg complete
    a.theta = a.target;
    a.vel = 0.0f;
    ++a.leg;
    // Even legs head out to FWD, odd legs come back to 0.
    a.target = (a.leg % 2 == 0) ? FWD : 0.0;
    a.dwell = DWELL_S;
    motorWriteProfile(a.theta_cmd, 0.0f, 0.0f, AMPS, a.m);
    return;
  }

  // Everything below works on unsigned SPEED and reapplies the sign at the
  // end, so the identical envelope serves both directions. a_cmd is the
  // derivative of the SIGNED velocity, hence the same dir factor.
  float dir = (remaining >= 0.0) ? 1.0f : -1.0f;
  float rem = (float)fabs(remaining);
  float vstop = sqrtf(2.0f * AMAX * rem);  // decel envelope
  float speed = fabsf(a.vel);
  float a_cmd;
  if (speed < VMAX) {
    speed += AMAX * dt;
    a_cmd = AMAX;
  } else {
    speed = VMAX;
    a_cmd = 0.0f;
  }
  if (speed > vstop) {
    speed = vstop;
    a_cmd = -AMAX;  // riding the envelope down at constant deceleration
  }

  a.vel = dir * speed;
  a.theta += (double)a.vel * dt;
  a.theta_cmd += a.vel * dt;
  if (a.theta_cmd >= 2.0f * PI)
    a.theta_cmd -= 2.0f * PI;
  else if (a.theta_cmd < 0.0f)
    a.theta_cmd += 2.0f * PI;

  motorWriteProfile(a.theta_cmd, a.vel, dir * a_cmd, AMPS, a.m);
}

void setup() {
  Serial.begin(115200);
  begin();
  for (int i = 0; i < N_AX; ++i) {
    if (!ax[i].on) continue;
    ax[i].target = FWD;
    motorEnable(ax[i].m);
  }
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

  if (!all_done) {
    bool any = false;
    for (int i = 0; i < N_AX; ++i)
      if (ax[i].on && legsRemain(ax[i])) any = true;
    if (!any) {
      all_done = true;
      for (int i = 0; i < N_AX; ++i)
        if (ax[i].on) motorDisable(ax[i].m);  // don't hold current forever
      Serial.println("# all cycles complete, motors disabled");
    }
  }

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
      Serial.print(",leg");
      Serial.print(n);
      Serial.print(':');
      Serial.print(ax[i].leg);
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
