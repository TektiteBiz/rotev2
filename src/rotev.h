#pragma once
#include "constants.h"
#include "profile.h"

namespace rotev {

void begin();

// --- Motor ---
void motorEnable(Motor m, bool enable = true);
void motorSetProfile(Motor m, const Profile& p);  // starts immediately from the current position
Profile      motorProfile(Motor m);   // the profile currently executing
ProfileState motorProgress(Motor m);  // where that profile is right now

// --- Board I/O ---
void  buzzerOn(uint16_t freq_hz);   // 1000-4000 Hz, 50% duty
void  buzzerOff();
void  ledColor(uint8_t r, uint8_t g, uint8_t b);
bool  buttonPressed(Button b);
float adcRead(AdcChannel ch);       // ADC_AIN1/2/3, volts
float busVoltage();                 // volts

}  // namespace rotev
