#include "foc_math.h"
#include <cmath>

namespace rotev {

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

DQ park(AB i, float theta_e) {
  float c = cosf(theta_e), s = sinf(theta_e);
  return { i.a * c + i.b * s, -i.a * s + i.b * c };
}

AB inversePark(float ud, float uq, float theta_e, float vbus) {
  float c = cosf(theta_e), s = sinf(theta_e);
  float va = ud * c - uq * s;   // volts
  float vb = ud * s + uq * c;
  float inv = (vbus > 0.0f) ? 1.0f / vbus : 0.0f;
  return { clampf(va * inv, -1.0f, 1.0f), clampf(vb * inv, -1.0f, 1.0f) };
}

void piReset(PIState& s) { s.integ = 0.0f; }

float piStep(PIState& s, float error, float kp, float ki, float dt, float out_limit) {
  float integ_next = s.integ + ki * error * dt;
  float unsat = kp * error + integ_next;
  float out = clampf(unsat, -out_limit, out_limit);
  // Anti-windup: only commit the integrator if we are not saturating further out.
  if (out == unsat) s.integ = integ_next;
  return out;
}

} // namespace rotev
