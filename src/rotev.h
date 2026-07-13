#pragma once
#include "constants.h"
namespace rotev {

void  begin();
void  motorEnable(Motor m);
void  motorDisable(Motor m);
void  motorWrite(float theta_rad, float amps, Motor m);
void  motorWriteVoltage(float theta_rad, float uq_volts, Motor m);
void  motorWriteVoltageAB(float va_volts, float vb_volts, Motor m);
void  buzzerOn(uint16_t freq_hz);  // clamped to [1000,4000] Hz, 50% duty
void  buzzerOff();
float adcRead(AdcChannel ch);  // ADC_AIN1/2/3, last cached sample in volts, non-blocking
float busVoltage();            // last cached bus voltage in volts, non-blocking
void  ledColor(uint8_t r, uint8_t g, uint8_t b);
bool  buttonPressed(Button b);
float motorCurrentA(Motor m);
float motorCurrentB(Motor m);

} // namespace rotev
