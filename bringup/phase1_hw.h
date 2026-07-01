#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);   // nSLEEP high; no motion commanded
  motorEnable(MOTOR_2);
}

void loop() {
  if (buttonPressed(BTN_GO))   ledColor(0, 255, 0);
  if (buttonPressed(BTN_STOP)) ledColor(255, 0, 0);
  Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
  Serial.print(motorCurrentB(MOTOR_1)); Serial.print(',');
  Serial.print(motorCurrentA(MOTOR_2)); Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_2));
  delay(50);
}
