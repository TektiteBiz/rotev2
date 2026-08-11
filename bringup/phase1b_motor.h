#pragma once
#include <Arduino.h>
#include <rotev.h>
#include <rotev_internal.h>
using namespace rotev;

// Open-loop sinusoidal drive through the FOC ISR: the inverter runs normally
// (correct ADC timing, telemetry populated) but the PI is bypassed -- uq is
// applied straight through the inverse Park. Verifies current-sense sign,
// amplitude and channel assignment before the current loop is closed.
//
// 5 RPM is slow enough that the winding is essentially resistive, so the two
// phase currents should be sinusoids ~90 deg apart with amplitude
// UQ_VOLTS / PHASE_R. If they come out inverted, fix it at the source with
// ISENSE_SCALE_x in constants.h rather than here.

static constexpr float RPM = 5.0f;
static constexpr float UQ_VOLTS = 3.0f;  // 0.25 of a 12 V bus

static float theta = 0.0f;
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
  theta += 2.0f * PI * (RPM / 60.0f) * dt;
  motorSetVoltage(MOTOR_1, theta, UQ_VOLTS);

  if (now - last_print_us >= 25000) {
    last_print_us = now;
    Serial.print(">sensA:"); Serial.print(motorCurrentA(MOTOR_1), 3);
    Serial.print(",sensB:"); Serial.println(motorCurrentB(MOTOR_1), 3);
  }

  delayMicroseconds(500);
}
