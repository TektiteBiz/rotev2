#pragma once
#include <Arduino.h>
#include <rotev.h>
#include <rotev_internal.h>
using namespace rotev;

// Open-loop sinusoidal drive at speed: a fixed VOLTAGE (not a current
// command) chosen to give ~0.7 A of phase current while spinning at 60 RPM
// mechanical -> 50 Hz electrical (60 * POLE_PAIRS / 60).
//
// The current target is 0.7 A, not the 0.1 A of the original plan: below
// ~0.5 A this motor never overcomes its own detent and static friction, so it
// buzzes in place instead of turning (confirmed on hardware). 0.7 A still
// sits well under IMAX_A (1.1 A).
//
// At 50 Hz electrical the reactance is no longer negligible next to the
// winding resistance, so the voltage has to be sized against |Z|:
//   we = 2*PI*50 = 314.16 rad/s, we*L = 1.10 ohm
//   |Z| = sqrt(3.5^2 + 1.10^2) = 3.6686 ohm   (datasheet R/L, pre-phase5)
//
// A stepper rotor cannot snap to 50 Hz electrical from standstill -- the
// field would outrun its pull-in torque and it would buzz in place -- so ramp
// the mechanical speed linearly to RPM before holding there.

static constexpr float TARGET_AMPS = 0.7f;
static constexpr float RPM = 60.0f;
static constexpr float UQ_VOLTS = TARGET_AMPS * 3.6686f;
static constexpr float RAMP_TIME_S = 0.01f;

static float theta = 0.0f;
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
  motorSetVoltage(MOTOR_1, theta, UQ_VOLTS);

  if (now - last_print_us >= 5000) {
    last_print_us = now;
    Serial.print(">sensA:"); Serial.print(motorCurrentA(MOTOR_1), 3);
    Serial.print(",sensB:"); Serial.println(motorCurrentB(MOTOR_1), 3);
  }

  delayMicroseconds(500);
}
