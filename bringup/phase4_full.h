#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Trapezoidal/S-curve position profile to 100 rotations, peak 300 RPM.
static const float TARGET_REV = 100.0f;
static const float VMAX = 500.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
static const float AMAX = VMAX / 0.25f;                // reach vmax in 0.5 s
static float theta = 0.0f, vel = 0.0f;
static uint32_t last_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  setLagComp(true);  // Phase 4: enable lag compensation
  motorEnable(MOTOR_1);
  last_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  float target = TARGET_REV * 2.0f * PI;
  float remaining = target - theta;
  float vstop = sqrtf(2.0f * AMAX * fabsf(remaining));  // decel envelope
  float vcmd = (vel < VMAX) ? vel + AMAX * dt : VMAX;
  if (vcmd > vstop) vcmd = vstop;
  vel = vcmd;
  theta += vel * dt;
  if (theta > target) theta = target;
  motorWrite(theta, 0.8f, MOTOR_1);
  Serial.print(motorCurrentA(MOTOR_1));
  Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_1));
  delayMicroseconds(500);
}
