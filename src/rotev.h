#pragma once
#include "constants.h"
namespace rotev {

void  begin();
void  motorEnable(Motor m);
void  motorDisable(Motor m);
void  motorWrite(float theta_rad, float amps, Motor m);
void  ledColor(uint8_t r, uint8_t g, uint8_t b);
bool  buttonPressed(Button b);
float motorCurrentA(Motor m);
float motorCurrentB(Motor m);

} // namespace rotev
