#include "foc_math.h"
#include "constants.h"
#ifdef ARDUINO
#include "pico.h"
#else
#define __not_in_flash_func(func_name) func_name
#endif
#include <cmath>
#include <cstdint>

namespace rotev {

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static constexpr float PI_F = 3.14159265358979323846f;

// Wraps to (-PI, PI]. sinf/cosf range-reduction cost grows and becomes
// data-dependent for large arguments, so callers must not feed unbounded
// accumulated angles into trig -- see electricalAngle().
static inline float wrapAngle(float a) {
  a = fmodf(a, 2.0f * PI_F);
  if (a > PI_F) a -= 2.0f * PI_F;
  else if (a <= -PI_F) a += 2.0f * PI_F;
  return a;
}

DQ __not_in_flash_func(park)(AB i, float theta_e) {
  float c = cosf(theta_e), s = sinf(theta_e);
  return { i.a * c + i.b * s, -i.a * s + i.b * c };
}

AB __not_in_flash_func(inversePark)(float ud, float uq, float theta_e, float vbus) {
  float c = cosf(theta_e), s = sinf(theta_e);
  float va = ud * c - uq * s;   // volts
  float vb = ud * s + uq * c;
  float inv = (vbus > 0.0f) ? 1.0f / vbus : 0.0f;
  return { clampf(va * inv, -1.0f, 1.0f), clampf(vb * inv, -1.0f, 1.0f) };
}

void piReset(PIState& s) { s.integ = 0.0f; }

float __not_in_flash_func(piStep)(PIState& s, float error, float kp, float ki, float dt, float out_limit) {
  float integ_next = s.integ + ki * error * dt;
  float unsat = kp * error + integ_next;
  float out = clampf(unsat, -out_limit, out_limit);
  // Anti-windup: only commit the integrator if we are not saturating further out.
  if (out == unsat) s.integ = integ_next;
  return out;
}

float electricalAngle(float theta_mech) { return wrapAngle(theta_mech * (float)POLE_PAIRS); }

float countsToAmps(uint16_t counts) {
  float v = ((float)counts / ADC_MAX) * ADC_VREF;
  return (v - ISENSE_REF_V) / VOLTS_PER_AMP;
}

uint16_t ledDuty(uint8_t value, uint16_t top) {
  uint32_t on = ((uint32_t)value * top) / 255u;   // desired brightness level
  return (uint16_t)(top - on);                      // active-low invert
}

uint16_t buzzClampFreq(uint16_t freq_hz) {
  if (freq_hz < BUZZ_MIN_HZ) return BUZZ_MIN_HZ;
  if (freq_hz > BUZZ_MAX_HZ) return BUZZ_MAX_HZ;
  return freq_hz;
}

float adsRawToVolts(int16_t raw16) {
  int16_t code12 = raw16 >> 4;  // arithmetic shift preserves sign
  return (float)code12 * (ADS1015_FSR_V / 2048.0f);
}

float dividerToVbus(float v_div) {
  return v_div * (VBUS_DIV_HIGH_OHMS + VBUS_DIV_LOW_OHMS) / VBUS_DIV_LOW_OHMS;
}

uint8_t adcSeqChannel(uint8_t seq_idx) {
  static constexpr uint8_t kSeq[6] = {0, 1, 0, 2, 0, 3};
  return kSeq[seq_idx % 6];
}

uint16_t adcConfigForChannel(uint8_t ch) {
  // OS=1, MUX=100+ch, PGA=001 (+-4.096V), MODE=1 (single-shot), DR=111 (3300SPS),
  // COMP_MODE=0, COMP_POL=0, COMP_LAT=0, COMP_QUE=11 (disabled)
  uint16_t mux = (uint16_t)(4 + ch);  // 100..111
  return (uint16_t)(0x8000 |            // OS
                     (mux << 12) |
                     (0x1 << 9)  |      // PGA = 001
                     (0x1 << 8)  |      // MODE = single-shot
                     (0x7 << 5)  |      // DR = 111 (3300SPS)
                     0x3);              // COMP_QUE = 11
}

} // namespace rotev
