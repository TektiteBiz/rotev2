#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// DC current hold: target 0.25 A into phase A, phase B = 0.
// No rotation, no transforms. Just PI + ADC + PWM.

static float    p1c_integ      = 0.0f;
static uint32_t p1c_last       = 0;
static uint32_t p1c_last_print = 0;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  ledColor(0, 80, 0);  // green
  p1c_last = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - p1c_last) * 1e-6f;
  p1c_last = now;

  float ia    = motorCurrentA(MOTOR_1);
  float error = 0.25f - ia;

  float next = p1c_integ + KI * error * dt;
  float ua   = KP * error + next;
  if (ua >  VBUS_V) ua =  VBUS_V;
  if (ua < -VBUS_V) ua = -VBUS_V;
  else p1c_integ = next;   // only integrate when not saturated

  motorWriteVoltageAB(ua, 0.0f, MOTOR_1);

  if (now - p1c_last_print >= 25000) {
    p1c_last_print = now;
    Serial.println(ia, 3);
  }

  delayMicroseconds(500);
}
