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
  const float v = A * (T - std::sqrt(disc)) * 0.5f;
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

float Profile::distance()    const { return vpk_ * (ta_ + tc_); }
float Profile::duration()    const { return 2.0f * ta_ + tc_; }
float Profile::maxVelocity() const { return vpk_; }
float Profile::maxAccel()    const { return (ta_ > 0.0f) ? vpk_ / ta_ : 0.0f; }
float Profile::accelTime()   const { return ta_; }
float Profile::cruiseTime()  const { return tc_; }
float Profile::decelTime()   const { return ta_; }
bool  Profile::valid()       const { return ta_ > 0.0f && vpk_ != 0.0f; }

ProfileState Profile::at(float t) const {
  ProfileState st{0.0f, 0.0f, 0.0f, 0.0f, true};
  if (!valid()) return st;

  const float T = duration();
  if (!(t > 0.0f)) t = 0.0f;  // also catches NaN
  if (t > T) t = T;
  st.t = t;

  const float a = vpk_ / ta_;          // signed ramp acceleration
  const float x_ramp = 0.5f * vpk_ * ta_;  // distance covered by one ramp
  if (t >= T) {
    st.pos  = distance();
    st.vel  = 0.0f;
    st.acc  = 0.0f;
    st.done = true;
  } else if (t < ta_) {
    st.vel  = a * t;
    st.pos  = 0.5f * a * t * t;
    st.acc  = a;
    st.done = false;
  } else if (t < ta_ + tc_) {
    st.pos  = x_ramp + vpk_ * (t - ta_);
    st.vel  = vpk_;
    st.acc  = 0.0f;
    st.done = false;
  } else {
    float u = t - ta_ - tc_;
    st.vel  = vpk_ - a * u;
    st.pos  = x_ramp + vpk_ * tc_ + vpk_ * u - 0.5f * a * u * u;
    st.acc  = -a;
    st.done = false;
  }
  return st;
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
