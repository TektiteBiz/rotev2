#pragma once
#include "constants.h"
#include "foc_math.h"
namespace rotev {
void focStart();
void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled);
void focSetVoltage(Motor m, float theta_mech, float uq_volts, bool enabled);
void focSetVoltageAB(Motor m, float va_duty, float vb_duty, bool enabled);
AB   focTelemetry(Motor m);
void focSetLagComp(bool on);
}
