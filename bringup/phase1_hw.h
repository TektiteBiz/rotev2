#pragma once
#include <Arduino.h>
#include <rotev.h>
#include <rotev_internal.h>
using namespace rotev;

// Basic hardware verification: wake the drivers, log idle phase currents and
// bus voltage, confirm the LED/buttons/buzzer work. No motion is commanded,
// so every current reading here is the sensor's zero -- that is the number
// ISENSE_OFFSET_x is calibrated from.

static void playGoTune() {
  const uint16_t notes[] = {1000, 1500, 2000};
  for (uint16_t f : notes) {
    buzzerOn(f);
    delay(80);
  }
  buzzerOff();
}

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);  // nSLEEP high; no motion commanded
  motorEnable(MOTOR_2);
}

void loop() {
  Serial.print(">iA1:");  Serial.print(motorCurrentA(MOTOR_1), 3);
  Serial.print(",iB1:");  Serial.print(motorCurrentB(MOTOR_1), 3);
  Serial.print(",iA2:");  Serial.print(motorCurrentA(MOTOR_2), 3);
  Serial.print(",iB2:");  Serial.print(motorCurrentB(MOTOR_2), 3);
  Serial.print(",vbus:"); Serial.println(busVoltage(), 2);

  if (buttonPressed(BTN_GO))   { ledColor(0, 255, 0); playGoTune(); }
  if (buttonPressed(BTN_STOP)) { ledColor(255, 0, 0); }
  delay(50);
}
