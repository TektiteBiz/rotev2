#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

static float theta = 0.0f;
static uint32_t last_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  setLagComp(false);        // Phase 3: no lag comp
  motorEnable(MOTOR_1);
  last_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f; last_us = now;
  theta += 2.0f * PI * dt;                 // constant velocity 60 RPM
  motorWrite(theta, 0.5f, MOTOR_1);        // higher torque current
  Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_1));
  delayMicroseconds(500);
}
