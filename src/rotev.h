#pragma once
#include "constants.h"
#include "profile.h"

namespace rotev {

void begin();

// --- Motor ---
void motorEnable(Motor m, bool enable = true);
void motorSetProfile(Motor m, const Profile& p);  // starts immediately from the current position
Profile      motorProfile(Motor m);   // the profile currently executing

// True if the axis has stopped driving because the bus-voltage sense went
// invalid or stale. LATCHED: the axis stays de-energised and the profile clock
// stays stopped until motorEnable() clears it, so motorProgress() will never
// report done. Poll this alongside .done or a move can hang forever.
bool  motorFault(Motor m);
ProfileState motorProgress(Motor m);  // where that profile is right now

// --- Board I/O ---
void  buzzerOn(uint16_t freq_hz);   // 1000-4000 Hz, 50% duty
void  buzzerOff();
void  ledColor(uint8_t r, uint8_t g, uint8_t b);
bool  buttonPressed(Button b);
float adcRead(AdcChannel ch);       // ADC_AIN1/2/3, volts
float busVoltage();                 // volts

}  // namespace rotev
