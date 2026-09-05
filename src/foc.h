#pragma once
#include "constants.h"
#include "foc_math.h"
#include "profile.h"

namespace rotev {

void focStart();

// Command modes. Setting one replaces whatever the motor was doing.
void focEnable(Motor m, bool enable);
void focSetProfile(Motor m, const Profile& p);  // restarts the profile clock at 0
void focSetVelocity(Motor m, float vel_rad_s);  // degenerate profile: cruise forever
void focSetVoltage(Motor m, float theta_mech, float uq_volts);
void focSetVoltageAB(Motor m, float va_duty, float vb_duty);

Profile      focProfile(Motor m);
ProfileState focProgress(Motor m);

AB focTelemetry(Motor m);    // measured phase currents
DQ focTelemetryU(Motor m);   // applied dq voltage, post-clamp
DQ focTelemetryI(Motor m);   // measured dq current
float focDerateTrim(Motor m);  // saturation trim on the derate, 1.0 = no cut
float focCurrentCmd(Motor m);  // slewed q-axis current command (move vs hold)
float focLqEstimate(Motor m);  // online inductance estimate feeding the derate
bool  focFault(Motor m);       // latched vbus fault; cleared by focEnable()

}  // namespace rotev
