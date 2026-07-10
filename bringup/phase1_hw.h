#pragma once
#include <Arduino.h>
#include <rotev.h>
#include "hw.h"
#include "led.h"
#include "pwm.h"
using namespace rotev;

void setup() {
  Serial.begin(115200);
  // Skip focStart so no core1 ISR overwrites PWM; init hardware directly.
  hwInit();
  ledInit();
  pwmInit();
  hwSetNsleep(MOTOR_1, true);
  hwSetNsleep(MOTOR_2, true);
  pwmSetPhase(PIN_ENA_1, PIN_PHA_1, 0.3f);
  pwmSetPhase(PIN_ENB_1, PIN_PHB_1, 0.3f);
}

void loop() {
  bool go   = buttonPressed(BTN_GO);
  bool stop = buttonPressed(BTN_STOP);

  Serial.print("BTN_GO="); Serial.print(go);
  Serial.print(" BTN_STOP="); Serial.print(stop);
  Serial.print(" PIN20="); Serial.print(digitalRead(PIN_BTN_GO));
  Serial.print(" PIN19="); Serial.println(digitalRead(PIN_BTN_STOP));

  if (go)   ledSet(0, 255, 0);
  if (stop) ledSet(255, 0, 0);
  delay(50);
}
