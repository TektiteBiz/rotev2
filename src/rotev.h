#pragma once
#include "constants.h"
namespace rotev {

void  begin();
void  motorEnable(Motor m);
void  motorDisable(Motor m);
void  motorWrite(float theta_rad, float amps, Motor m);
void  motorWriteVelocity(float vel_rad_s, float amps, Motor m);  // ISR integrates the angle
void  motorWriteProfile(float theta_rad, float vel_rad_s, float acc_rad_s2, float amps, Motor m);  // position + vel/acc feedforward
void  motorWriteVoltage(float theta_rad, float uq_volts, Motor m);
void  motorWriteVoltageAB(float va_volts, float vb_volts, Motor m);
void  buzzerOn(uint16_t freq_hz);  // clamped to [1000,4000] Hz, 50% duty
void  buzzerOff();
float adcRead(AdcChannel ch);  // ADC_AIN1/2/3, last cached sample in volts, non-blocking
float busVoltage();            // last cached bus voltage in volts, non-blocking
void  ledColor(uint8_t r, uint8_t g, uint8_t b);
bool  buttonPressed(Button b);
float motorDerate(Motor m);    // voltage-limited iq scaling, 1.0 = none
float motorVoltageD(Motor m);  // applied d-axis volts
float motorVoltageQ(Motor m);  // applied q-axis volts
float motorCurrentD(Motor m);  // measured d-axis amps
float motorCurrentQ(Motor m);  // measured q-axis amps
float motorCurrentA(Motor m);
float motorCurrentB(Motor m);

} // namespace rotev
