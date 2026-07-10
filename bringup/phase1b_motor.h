#pragma once
#include <Arduino.h>
#include <rotev.h>

#include "foc_math.h"
#include "hw.h"
#include "led.h"
#include "pwm.h"
using namespace rotev;

// Open-loop sinusoidal drive with NO current sensing and NO FOC ISR.
// Purpose: confirm motor moves and direction is correct before closing the
// current loop.
//
// Tunable knobs:
//   RPM       -- mechanical speed
//   DUTY_PEAK -- peak duty cycle [0, 1] relative to Vbus (0.3 = 3.6 V at 12 V
//   bus)

static constexpr float RPM = 5.0f;
static constexpr float DUTY_PEAK = 0.3f;

static float theta = 0.0f;
static uint32_t last_us = 0;
static uint32_t last_print_us = 0;

void setup() {
  Serial.begin(115200);
  hwInit();
  ledInit();
  pwmInit();
  hwSetNsleep(MOTOR_1, true);
  last_us = last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  theta += 2.0f * PI * (RPM / 60.0f) * dt;

  float theta_e = electricalAngle(theta);
  float va = DUTY_PEAK * sinf(theta_e);
  float vb = DUTY_PEAK * cosf(theta_e);
  pwmSetPhase(PIN_ENA_1, PIN_PHA_1, va);
  pwmSetPhase(PIN_ENB_1, PIN_PHB_1, vb);

  // Print commanded duties so the Serial Plotter shows two clean
  // 90-degree-offset sines.
  if (now - last_print_us >= 25000) {
    last_print_us = now;
    Serial.print(">A:");
    Serial.print(va);
    Serial.print(",B:");
    Serial.println(vb);
  }

  delayMicroseconds(500);
}
