#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Open-loop sinusoidal drive, per PRD: a fixed VOLTAGE (not a current
// command) chosen so the resulting phase current is ~0.1 A, spinning at
// 60 RPM mechanical -> 50 Hz electrical (60 * POLE_PAIRS / 60).
// Uses motorWriteVoltage -- the same open-loop path proven in phase1b/1c --
// with a continuously rotating theta instead of a fixed one.
//
// At 50 Hz electrical the inductive reactance is no longer negligible next
// to the winding resistance: |Z| = sqrt(R^2 + (omega_e*L)^2)
//   omega_e = 2*PI*50 = 314.16 rad/s, omega_e*L = 1.10 ohm
//   |Z| = sqrt(3.5^2 + 1.10^2) = 3.67 ohm
// Target 0.1 A -> UQ_VOLTS = 0.1 * 3.67 = 0.367 V.
//
// A real stepper rotor cannot instantly snap to 50 Hz electrical from a
// standstill -- the field would outrun the rotor's pull-in torque and it
// would just buzz in place instead of synchronizing. Ramp mechanical speed
// linearly from 0 to RPM over RAMP_TIME_S before holding at RPM.
static constexpr float RPM          = 60.0f;
static constexpr float UQ_VOLTS     = 0.367f;
static constexpr float RAMP_TIME_S  = 2.0f;

static float theta   = 0.0f;
static float elapsed = 0.0f;
static uint32_t last_us = 0;
static uint32_t last_print_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  last_us = last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  elapsed += dt;
  float rpm_now = (elapsed < RAMP_TIME_S) ? RPM * (elapsed / RAMP_TIME_S) : RPM;
  theta += 2.0f * PI * (rpm_now / 60.0f) * dt;
  motorWriteVoltage(theta, UQ_VOLTS, MOTOR_1);

  if (now - last_print_us >= 5000) {
    last_print_us = now;
    Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
    Serial.println(motorCurrentB(MOTOR_1));
  }

  delayMicroseconds(500);
}
