#pragma once
#include <cstdint>

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

// Electrical angle from mechanical angle (scaled by POLE_PAIRS).
float electricalAngle(float theta_mech);

// First-order LPF electrical speed estimator.
struct OmegaEst { float prev_theta_e; float w_filt; bool primed; };
void  omegaReset(OmegaEst& s);
float omegaStep(OmegaEst& s, float theta_e, float dt, float alpha);

// ADC current sense conversion and clamping.
float    countsToAmps(uint16_t counts);
float    clampCurrent(float amps);

// Active-low LED duty cycle: returns top - (value*top)/255.
uint16_t ledDuty(uint8_t value, uint16_t top);

} // namespace rotev
