#pragma once
#include "constants.h"
#include "foc_math.h"
namespace rotev {
void focStart();
void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled);
void focSetVelocity(Motor m, float vel_rad_s, float iq_cmd, bool enabled);
void focSetProfile(Motor m, float theta_mech, float vel_rad_s, float iq_cmd, bool enabled);
void focSetVoltage(Motor m, float theta_mech, float uq_volts, bool enabled);
void focSetVoltageAB(Motor m, float va_duty, float vb_duty, bool enabled);
AB   focTelemetry(Motor m);
float focDerate(Motor m);      // voltage-limited iq scaling, 1.0 = none
DQ   focTelemetryU(Motor m);   // applied dq voltage, post-clamp
DQ   focTelemetryI(Motor m);   // measured dq current
}
