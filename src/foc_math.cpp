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

} // namespace rotev
