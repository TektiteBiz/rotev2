#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Basic hardware verification: enable/disable the drivers, log idle phase
// currents, and confirm the LED/buttons work. No PWM is applied to the
// motor windings beyond what the driver requires to acknowledge enable.

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);   // nSLEEP high; no motion commanded
  motorEnable(MOTOR_2);
}

void loop() {
  bool go   = buttonPressed(BTN_GO);
  bool stop = buttonPressed(BTN_STOP);

  Serial.print(">motorCurrentA(MOTOR_1):"); Serial.print(motorCurrentA(MOTOR_1));
  Serial.print(",motorCurrentB(MOTOR_1):"); Serial.print(motorCurrentB(MOTOR_1));
  Serial.print(",motorCurrentA(MOTOR_2):"); Serial.print(motorCurrentA(MOTOR_2));
  Serial.print(",motorCurrentB(MOTOR_2):"); Serial.println(motorCurrentB(MOTOR_2));

  if (go)   ledColor(0, 255, 0);
  if (stop) ledColor(255, 0, 0);
  delay(50);
}
