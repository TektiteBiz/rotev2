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

void omegaReset(OmegaEst& s) { s.prev_theta_e = 0.0f; s.w_filt = 0.0f; s.primed = false; }

float omegaStep(OmegaEst& s, float theta_e, float dt, float alpha) {
  if (!s.primed) { s.prev_theta_e = theta_e; s.primed = true; return 0.0f; }
  // theta_e is wrapped to (-PI, PI], so a plain difference across the wrap
  // boundary would show a false ~2*PI jump -- wrap the delta itself to the
  // shortest angular distance instead.
  float raw = wrapAngle(theta_e - s.prev_theta_e) / dt;
  s.prev_theta_e = theta_e;
  s.w_filt += alpha * (raw - s.w_filt);
  return s.w_filt;
}

float countsToAmps(uint16_t counts) {
  float v = ((float)counts / ADC_MAX) * ADC_VREF;
  return (v - ISENSE_REF_V) / VOLTS_PER_AMP;
}

float clampCurrent(float amps) { return clampf(amps, -IMAX_A, IMAX_A); }

uint16_t ledDuty(uint8_t value, uint16_t top) {
  uint32_t on = ((uint32_t)value * top) / 255u;   // desired brightness level
  return (uint16_t)(top - on);                      // active-low invert
}

} // namespace rotev
