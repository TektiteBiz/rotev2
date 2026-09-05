#pragma once
#include "constants.h"

namespace rotev {

// Open-loop / characterization primitives (bringup phases 1b, 1c, 2, 5).
void  motorSetVoltage(Motor m, float theta_rad, float uq_volts);
void  motorSetVoltageAB(Motor m, float va_volts, float vb_volts);
void  motorSetVelocity(Motor m, float vel_rad_s);  // closed loop, cruise forever

// Telemetry.
float motorCurrentA(Motor m);
float motorCurrentB(Motor m);
float motorCurrentD(Motor m);
float motorCurrentQ(Motor m);
float motorVoltageD(Motor m);
float motorVoltageQ(Motor m);
float motorDerateTrim(Motor m);  // 1.0 = feedforward untrimmed, <1 = riding the voltage limit
float motorCurrentCmd(Motor m);  // commanded iq after the move/hold slew
float motorLqEstimate(Motor m);  // the s_lq the derate is actually using, H

}  // namespace rotev
