#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Closed-loop FOC at constant velocity.
//
// controlStep() has no position sensor -- it takes the COMMANDED theta as a
// stand-in for the true rotor position and uses it to park-transform the
// measured current for feedback. At power-up the real rotor angle is
// arbitrary (nothing has ever aligned it), so that assumption is false from
// sample 0 unless we force it to be true first: hold a fixed theta with a
// modest current for ALIGN_TIME_S so the rotor physically settles into that
// position before the closed loop starts depending on the assumption.
// (Phase 2 doesn't need this -- open-loop voltage control never looks at
// measured current to decide its output, so it can't be fed a bad feedback
// projection the way the closed loop can.)
static constexpr float RPM = 300.0f;
static constexpr float TARGET_AMPS = 0.5f;
static constexpr float ALIGN_TIME_S = 0.5f;
static constexpr float RAMP_TIME_S = 2.0f;

static float theta = 0.0f;
static float elapsed = 0.0f;
static uint32_t last_us = 0;
static uint32_t last_print_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  setLagComp(true);  // Phase 3: no lag comp
  motorEnable(MOTOR_1);
  last_us = last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  elapsed += dt;

  if (elapsed < ALIGN_TIME_S) {
    motorWrite(0.0f, TARGET_AMPS,
               MOTOR_1);  // hold rotor at theta=0 to align it
  } else {
    float ramp_elapsed = elapsed - ALIGN_TIME_S;
    float rpm_now =
        (ramp_elapsed < RAMP_TIME_S) ? RPM * (ramp_elapsed / RAMP_TIME_S) : RPM;
    theta += 2.0f * PI * (rpm_now / 60.0f) * dt;
    motorWrite(theta, TARGET_AMPS, MOTOR_1);
  }

  if (now - last_print_us >= 25000) {
    last_print_us = now;
    Serial.print(">sensA:");
    Serial.print(motorCurrentA(MOTOR_1));
    Serial.print(",sensB:");
    Serial.println(motorCurrentB(MOTOR_1));
  }

  delayMicroseconds(500);
}
