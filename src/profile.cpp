#include "profile.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cstdio>
#endif
#include <cmath>

namespace rotev {

static constexpr float PI_F = 3.14159265358979323846f;

static inline bool finitePos(float x) { return std::isfinite(x) && x > 0.0f; }

Profile::Profile() : vpk_(0.0f), ta_(0.0f), tc_(0.0f) {}

Profile Profile::fromVelAccel(float distance, float max_vel, float max_accel) {
  Profile p;
  const float s = (distance < 0.0f) ? -1.0f : 1.0f;
  const float D = std::fabs(distance), V = std::fabs(max_vel), A = std::fabs(max_accel);
  if (!finitePos(D) || !finitePos(V) || !finitePos(A)) return p;

  const float ta = V / A;
  if (V * ta <= D) {
    // Trapezoidal: the two ramps together cover V*ta, the rest is cruise at V.
    p.vpk_ = s * V;
    p.ta_  = ta;
    p.tc_  = D / V - ta;
  } else {
    // Triangular: V is never reached, so solve vpk*ta == D with ta = vpk/A.
    const float vpk = std::sqrt(A * D);
    p.vpk_ = s * vpk;
    p.ta_  = vpk / A;
    p.tc_  = 0.0f;
  }
  if (!std::isfinite(p.vpk_) || !finitePos(p.ta_) || !std::isfinite(p.tc_)) return Profile();
  if (p.tc_ < 0.0f) p.tc_ = 0.0f;  // rounding only; the branch above rules out real negatives
  return p;
}

Profile Profile::fromTimeAccel(float distance, float time, float max_accel) {
  const float D = std::fabs(distance), A = std::fabs(max_accel), T = time;
  if (!finitePos(D) || !finitePos(T) || !finitePos(A)) return Profile();

  // Duration as a function of peak velocity v is T(v) = v/A + D/v, so v solves
  // v^2/A - T v + D = 0. The smaller root is the slow branch that still
  // cruises; the larger one would exceed max_accel.
  float disc = T * T - 4.0f * D / A;
  if (!(disc >= 0.0f)) {
    // T is below the time-optimal move at this accel: give the fastest one.
    return fromVelAccel(distance, std::sqrt(A * D), A);
  }
  // 2D/(T+sqrt(disc)), not A*(T-sqrt(disc))/2. The roots multiply to A*D, so
  // the two forms are algebraically identical -- but the subtraction cancels
  // catastrophically once T*T swamps 4D/A. fromTimeAccel(0.1, 300, 100) has
  // ulp(T*T) = 0.0078 against a 4D/A of 0.004, so disc rounds back to T*T,
  // sqrt returns exactly T, and v came out 0: a silently empty profile for a
  // request sitting 4700x above the minimum time. This form has no subtraction.
  const float v = 2.0f * D / (T + std::sqrt(disc));
  if (!finitePos(v)) return Profile();
  return fromVelAccel(distance, v, A);
}

Profile Profile::scaleDistance(float k) const {
  Profile p;
  if (!valid() || !std::isfinite(k) || k == 0.0f) return p;
  p.vpk_ = vpk_ * k;
  p.ta_  = ta_;
  p.tc_  = tc_;
  return p;
}

Profile Profile::scaleTime(float k) const {
  Profile p;
  if (!valid() || !finitePos(k)) return p;
  p.vpk_ = vpk_ / k;
  p.ta_  = ta_ * k;
  p.tc_  = tc_ * k;
  return p;
}

void Profile::print() const {
#ifdef ARDUINO
  if (!valid()) { Serial.println("Profile: <empty>"); return; }
  Serial.printf("Profile: %.3f rad in %.3f s\n", distance(), duration());
  Serial.printf("  peak vel   %.3f rad/s (%.1f rpm)\n", maxVelocity(),
                maxVelocity() * 60.0f / (2.0f * PI_F));
  Serial.printf("  peak accel %.3f rad/s^2\n", maxAccel());
  Serial.printf("  accel      %.3f s   cruise %.3f s   decel %.3f s\n",
                accelTime(), cruiseTime(), decelTime());
#else
  if (!valid()) { std::printf("Profile: <empty>\n"); return; }
  std::printf("Profile: %.3f rad in %.3f s\n", (double)distance(), (double)duration());
  std::printf("  peak vel   %.3f rad/s (%.1f rpm)\n", (double)maxVelocity(),
              (double)(maxVelocity() * 60.0f / (2.0f * PI_F)));
  std::printf("  peak accel %.3f rad/s^2\n", (double)maxAccel());
  std::printf("  accel      %.3f s   cruise %.3f s   decel %.3f s\n",
              (double)accelTime(), (double)cruiseTime(), (double)decelTime());
#endif
}

}  // namespace rotev
