#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Open-loop sinusoidal drive via the FOC ISR.
// The inverter runs normally (correct ADC timing, telemetry populated) but
// the PI is bypassed — uq_volts is applied directly through inverse-Park.
// Use this to verify current sense sign, amplitude, and channel assignment
// before closing the current loop in phase 2.
//
// Tunable knobs:
//   RPM      -- mechanical speed
//   UQ_VOLTS -- peak q-axis voltage (3.6 V = 30% of 12 V bus)

static constexpr float RPM = 5.0f;
static constexpr float UQ_VOLTS =
    3.0f;  // 0.3 * VBUS_V, matches old DUTY_PEAK = 0.3

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

  motorWriteVoltage(theta, UQ_VOLTS, MOTOR_1);

  if (now - last_print_us >= 25000) {
    last_print_us = now;
    // sensA/sensB are the ISR-timed ADC readings in amps.
    // Expected: two sinusoids ~90 deg apart, amplitude ~ UQ_VOLTS / PHASE_R.
    // If inverted vs expected direction: negate in adc.cpp (ISENSE-SIGN knob).
    Serial.print(">sensA:");
    Serial.print(motorCurrentA(MOTOR_1));
    Serial.print(",sensB:");
    Serial.println(motorCurrentB(MOTOR_1));
  }

  delayMicroseconds(500);
}
