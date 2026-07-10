#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// 60 RPM mechanical -> 50 Hz electrical.
// Print at 5 ms -> 200 Hz sample rate -> ~4 samples per electrical cycle.
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
  theta += 2.0f * PI * (5.0f / 60.0f) * dt;
  motorWrite(theta, 0.1f, MOTOR_1);

  if (now - last_print_us >= 5000) {
    last_print_us = now;
    Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
    Serial.println(motorCurrentB(MOTOR_1));
  }

  delayMicroseconds(500);
}
