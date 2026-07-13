#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Open-loop sinusoidal drive: a fixed VOLTAGE (not a current command) chosen
// so the resulting phase current is ~0.5 A, spinning at 60 RPM mechanical ->
// 50 Hz electrical (60 * POLE_PAIRS / 60). Uses motorWriteVoltage -- the same
// open-loop path proven in phase1b/1c -- with a continuously rotating theta
// instead of a fixed one.
//
// Target current is 0.5 A, not the original 0.1 A spec: at 0.1 A this motor
// never overcomes its own detent/static-friction torque, so it just buzzes
// in place instead of rotating, even with a correct startup ramp (confirmed
// on hardware). 0.5 A gives enough torque margin for clean rotation while
// staying well under IMAX_A (1.1 A) and phase1b's already-proven ~0.86 A.
//
// At 50 Hz electrical the inductive reactance is no longer negligible next
// to the winding resistance: |Z| = sqrt(R^2 + (omega_e*L)^2)
//   omega_e = 2*PI*50 = 314.16 rad/s, omega_e*L = 1.10 ohm
//   |Z| = sqrt(3.5^2 + 1.10^2) = 3.67 ohm
// Target 0.5 A -> UQ_VOLTS = 0.5 * 3.67 = 1.83 V.
//
// A real stepper rotor cannot instantly snap to 50 Hz electrical from a
// standstill -- the field would outrun the rotor's pull-in torque and it
// would just buzz in place instead of synchronizing. Ramp mechanical speed
// linearly from 0 to RPM over RAMP_TIME_S before holding at RPM.
static constexpr float TARGET_AMPS = 0.7f;
static constexpr float RPM = 60.0f;
static constexpr float UQ_VOLTS =
    TARGET_AMPS * 3.6686f;  // I * sqrt(R^2+(we*L)^2)
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
  motorWriteVoltage(theta, UQ_VOLTS, MOTOR_1);

  if (now - last_print_us >= 5000) {
    last_print_us = now;
    Serial.print(">sensA:");
    Serial.print(motorCurrentA(MOTOR_1));
    Serial.print(",sensB:");
    Serial.println(motorCurrentB(MOTOR_1));
  }

  delayMicroseconds(500);
}
