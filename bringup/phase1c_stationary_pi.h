#pragma once
#include <Arduino.h>
#include <rotev.h>
#include <rotev_internal.h>
using namespace rotev;

// Stationary DC current test: the same open-loop voltage path as phase1b, but
// at a FIXED electrical angle, so all of the commanded voltage lands on phase
// A instead of sweeping across A/B.
//
// inversePark(ud=0, uq, theta_e): va = -uq*sin(theta_e), vb = uq*cos(theta_e),
// so theta_e = -90 deg gives va = +uq. theta_e = theta_mech * POLE_PAIRS.
//
// At DC the winding is purely resistive, so the current is just V/R: 0.875 V
// over the measured PHASE_R of 4.04 ohm -> ~0.22 A on sensA, ~0 on sensB.
// A negative sensA here is what the ISENSE_SCALE_A sign flip corrects.

static constexpr float THETA_MECH_PHASE_A = -PI / (2.0f * POLE_PAIRS);
static constexpr float UQ_VOLTS = 0.875f;

static uint32_t last_print_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  ledColor(0, 80, 0);
  motorSetVoltage(MOTOR_1, THETA_MECH_PHASE_A, UQ_VOLTS);
}

void loop() {
  uint32_t now = micros();
  if (now - last_print_us >= 25000) {
    last_print_us = now;
    Serial.print(">sensA:"); Serial.print(motorCurrentA(MOTOR_1), 3);
    Serial.print(",sensB:"); Serial.println(motorCurrentB(MOTOR_1), 3);
  }
}
