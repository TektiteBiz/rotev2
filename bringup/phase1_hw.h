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
  bool go   = buttonPressed(BTN_GO);
  bool stop = buttonPressed(BTN_STOP);

  Serial.print("BTN_GO="); Serial.print(go);
  Serial.print(" BTN_STOP="); Serial.print(stop);
  Serial.print(" PIN20="); Serial.print(digitalRead(PIN_BTN_GO));
  Serial.print(" PIN19="); Serial.print(digitalRead(PIN_BTN_STOP));
  Serial.print("  I1A="); Serial.print(motorCurrentA(MOTOR_1));
  Serial.print(" I1B="); Serial.print(motorCurrentB(MOTOR_1));
  Serial.print(" I2A="); Serial.print(motorCurrentA(MOTOR_2));
  Serial.print(" I2B="); Serial.println(motorCurrentB(MOTOR_2));

  if (go)   ledColor(0, 255, 0);
  if (stop) ledColor(255, 0, 0);
  delay(50);
}
