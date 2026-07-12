#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Stationary DC current test: reuses the exact open-loop voltage path proven
// working in phase1b (motorWriteVoltage / focSetVoltage), just with a FIXED
// electrical angle instead of a rotating one, so the commanded voltage lands
// entirely on phase A instead of sweeping sinusoidally across A/B.
//
// inversePark(ud=0, uq, theta_e): va = -uq*sin(theta_e), vb = uq*cos(theta_e).
// theta_e = -90 deg -> va = uq, vb = 0.  theta_e = theta_mech * POLE_PAIRS,
// so theta_mech = -PI / (2 * POLE_PAIRS) gives theta_e = -90 deg.
//
// No PI, no ab_mode, no integrator -- nothing here can wind up or saturate.
// At DC the winding is purely resistive, so V = I * R predicts the current:
// 0.875 V / 3.5 ohm -> ~0.25 A.

static constexpr float THETA_MECH_PHASE_A = -3.14159265f / (2.0f * POLE_PAIRS);
static constexpr float UQ_VOLTS = 0.875f;  // 0.25 A * 3.5 ohm

static uint32_t p1c_last_print = 0;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  ledColor(0, 80, 0);  // green
}

void loop() {
  motorWriteVoltage(THETA_MECH_PHASE_A, UQ_VOLTS, MOTOR_1);

  uint32_t now = micros();
  if (now - p1c_last_print >= 25000) {
    p1c_last_print = now;
    Serial.print(">sensA:");
    Serial.print(motorCurrentA(MOTOR_1), 3);
    Serial.print(",sensB:");
    Serial.println(motorCurrentB(MOTOR_1), 3);
  }

  delayMicroseconds(500);
}
