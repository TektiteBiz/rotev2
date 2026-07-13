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

// Buzzer frequency clamp (see constants.h BUZZ_MIN_HZ/BUZZ_MAX_HZ).
uint16_t buzzClampFreq(uint16_t freq_hz);

// ADS1015 raw 16-bit register value (12-bit result left-justified) -> volts at
// the ±4.096V PGA setting used for every channel.
float adsRawToVolts(int16_t raw16);

// Undoes the 7.3k/2.2k bus-voltage divider to recover the actual bus voltage.
float dividerToVbus(float v_div);

// Weighted round-robin channel sequence for the ADS1015 background sampler:
// AIN0 gets 2 of every 3 slots (bus voltage needs ~1kHz; user channels don't).
// Returns 0=AIN0, 1=AIN1, 2=AIN2, 3=AIN3 for sequence index seq_idx (wraps mod 6).
uint8_t adcSeqChannel(uint8_t seq_idx);

// ADS1015 single-shot config register value (16-bit) to start a conversion on
// the given channel (0=AIN0..3=AIN3): OS=1 (start), PGA=+-4.096V, MODE=single-shot,
// DR=3300SPS, COMP_QUE=11 (comparator disabled).
uint16_t adcConfigForChannel(uint8_t ch);

} // namespace rotev
