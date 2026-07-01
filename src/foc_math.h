#pragma once
namespace rotev {

struct DQ { float d, q; };
struct AB { float a, b; };

// Park: measured alpha-beta current -> dq at electrical angle theta_e.
DQ park(AB i, float theta_e);

// Inverse Park: dq voltage command -> normalized alpha-beta phase duty in [-1,1].
AB inversePark(float ud, float uq, float theta_e, float vbus);

struct PIState { float integ; };
void  piReset(PIState& s);
float piStep(PIState& s, float error, float kp, float ki, float dt, float out_limit);

} // namespace rotev
