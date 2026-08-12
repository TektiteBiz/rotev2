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

Profile::Profile() : vpk_(0.0f), ta_(0.0f), tc_(0.0f), td_(0.0f) {}

Profile Profile::fromVelAccel(float distance, float max_vel, float max_accel) {
  return fromAccelDecel(distance, max_vel, max_accel, max_accel);
}

Profile Profile::fromAccelDecel(float distance, float max_vel, float max_accel,
                                float max_decel) {
  Profile p;
  const float s = (distance < 0.0f) ? -1.0f : 1.0f;
  const float D = std::fabs(distance), V = std::fabs(max_vel);
  const float A = std::fabs(max_accel), Dc = std::fabs(max_decel);
  if (!finitePos(D) || !finitePos(V) || !finitePos(A) || !finitePos(Dc)) return p;

  // Both ramps together cover V^2/2 * (1/A + 1/Dc); call that 2*k*V^2 with
  // k = (1/A + 1/Dc)/2, the mean ramp time per unit velocity. The symmetric
  // case collapses to the familiar V*ta.
  const float k = 0.5f * (1.0f / A + 1.0f / Dc);
  if (k * V * V <= D) {
    // Trapezoidal: whatever the ramps do not cover is cruised at V.
    p.vpk_ = s * V;
    p.ta_  = V / A;
    p.td_  = V / Dc;
    p.tc_  = D / V - k * V;
  } else {
    // Triangular: V is never reached, so solve k*vpk^2 == D.
    const float vpk = std::sqrt(D / k);
    p.vpk_ = s * vpk;
    p.ta_  = vpk / A;
    p.td_  = vpk / Dc;
    p.tc_  = 0.0f;
  }
  if (!std::isfinite(p.vpk_) || !finitePos(p.ta_) || !finitePos(p.td_) ||
      !std::isfinite(p.tc_)) return Profile();
  if (p.tc_ < 0.0f) p.tc_ = 0.0f;  // rounding only; the branch above rules out real negatives
  return p;
}

Profile Profile::fromTimeAccel(float distance, float time, float max_accel) {
  return fromTimeAccelDecel(distance, time, max_accel, max_accel);
}

Profile Profile::fromTimeAccelDecel(float distance, float time, float max_accel,
                                    float max_decel) {
  const float D = std::fabs(distance), T = time;
  const float A = std::fabs(max_accel), Dc = std::fabs(max_decel);
  if (!finitePos(D) || !finitePos(T) || !finitePos(A) || !finitePos(Dc)) return Profile();

  // Duration as a function of peak velocity v is T(v) = k*v + D/v, where
  // k = (1/A + 1/Dc)/2 is the mean ramp time per unit velocity (1/A when the
  // ramps are symmetric). So v solves k v^2 - T v + D = 0. The smaller root is
  // the slow branch that still cruises; the larger one would exceed the limits.
  const float k = 0.5f * (1.0f / A + 1.0f / Dc);
  float disc = T * T - 4.0f * k * D;
  if (!(disc >= 0.0f)) {
    // T is below the time-optimal move at these rates: give the fastest one.
    return fromAccelDecel(distance, std::sqrt(D / k), A, Dc);
  }
  // 2D/(T+sqrt(disc)), not (T-sqrt(disc))/(2k). The roots multiply to D/k, so
  // the two forms are algebraically identical -- but the subtraction cancels
  // catastrophically once T*T swamps 4kD. fromTimeAccel(0.1, 300, 100) has
  // ulp(T*T) = 0.0078 against a 4kD of 0.004, so disc rounds back to T*T,
  // sqrt returns exactly T, and v came out 0: a silently empty profile for a
  // request sitting 4700x above the minimum time. This form has no subtraction.
  const float v = 2.0f * D / (T + std::sqrt(disc));
  if (!finitePos(v)) return Profile();
  return fromAccelDecel(distance, v, A, Dc);
}

// Ramp time per unit of cruise velocity, and the velocity at which the leg's
// two ramps meet with no cruise left between them.
static inline float legRampSlope(const Leg& l) {
  return 1.0f / std::fabs(l.accel) + 1.0f / std::fabs(l.decel);
}
static inline float legTriangleVel(const Leg& l, float slope) {
  return std::sqrt(2.0f * std::fabs(l.dist) / slope);
}

// Duration of one leg at a shared cruise velocity v. The clamp is the whole
// point: a leg whose ramps cannot reach v is a triangle, so its duration stops
// responding to v entirely. Crediting it with the trapezoid formula past that
// point keeps adding time that the leg will never spend, which is exactly how
// a solved velocity ends up missing the requested total.
static float legDuration(const Leg& l, float v) {
  const float D = std::fabs(l.dist), s = legRampSlope(l);
  const float v_tri = legTriangleVel(l, s);
  return (v >= v_tri) ? s * v_tri : 0.5f * v * s + D / v;
}

bool Profile::fromLegs(const Leg* legs, int n, float time, Profile* out) {
  if (!legs || !out || n <= 0) return false;
  for (int i = 0; i < n; ++i) {
    if (!finitePos(std::fabs(legs[i].dist)) || !finitePos(std::fabs(legs[i].accel)) ||
        !finitePos(std::fabs(legs[i].decel))) {
      for (int j = 0; j < n; ++j) out[j] = Profile();
      return false;
    }
  }

  // Flat out, every leg is its own triangle: that is the fastest the sequence
  // can go, and the upper bound on any useful cruise velocity.
  float v_hi = 0.0f, t_min = 0.0f, sum_half_slope = 0.0f, sum_dist = 0.0f;
  for (int i = 0; i < n; ++i) {
    const float s = legRampSlope(legs[i]);
    const float v_tri = legTriangleVel(legs[i], s);
    if (v_tri > v_hi) v_hi = v_tri;
    t_min          += s * v_tri;
    sum_half_slope += 0.5f * s;
    sum_dist       += std::fabs(legs[i].dist);
  }

  if (!(time > t_min)) {  // also catches NaN
    for (int i = 0; i < n; ++i) {
      out[i] = fromAccelDecel(legs[i].dist, legTriangleVel(legs[i], legRampSlope(legs[i])),
                              legs[i].accel, legs[i].decel);
    }
    return false;
  }

  // Closed form, assuming every leg cruises: sum(v*s/2 + D/v) == time is a
  // quadratic in v. Written as 2*sum_dist/(time+sqrt(disc)) rather than
  // (time-sqrt(disc))/(2*sum_half_slope) for the same reason fromTimeAccelDecel
  // is -- the subtraction cancels catastrophically for a generous budget.
  const float disc = time * time - 4.0f * sum_half_slope * sum_dist;
  float v = (disc >= 0.0f) ? 2.0f * sum_dist / (time + std::sqrt(disc)) : v_hi;

  bool saturated = !finitePos(v);
  for (int i = 0; i < n && !saturated; ++i) {
    if (v >= legTriangleVel(legs[i], legRampSlope(legs[i]))) saturated = true;
  }

  if (saturated) {
    // At least one leg is pinned triangular, so the quadratic was solving a
    // total that leg never contributes to. Bisect the true total instead: every
    // term is decreasing or flat in v, so the sum is monotone non-increasing
    // and the root is unique -- no local minima to trap a bracketing search.
    float lo = sum_dist / time;  // the speed if the ramps took no time at all
    for (int g = 0; g < 8; ++g) {
      float t = 0.0f;
      for (int i = 0; i < n; ++i) t += legDuration(legs[i], lo);
      if (t >= time) break;      // lo is genuinely below the root
      lo *= 0.5f;                // a very short leg saturated even down here
    }
    float hi = v_hi;
    for (int it = 0; it < 40; ++it) {
      const float mid = 0.5f * (lo + hi);
      float t = 0.0f;
      for (int i = 0; i < n; ++i) t += legDuration(legs[i], mid);
      if (t > time) lo = mid; else hi = mid;
    }
    v = 0.5f * (lo + hi);
  }

  for (int i = 0; i < n; ++i) {
    // A saturated leg gets a v it cannot reach; fromAccelDecel() clamps it to
    // the triangle on its own, which is the same duration legDuration() used.
    out[i] = fromAccelDecel(legs[i].dist, v, legs[i].accel, legs[i].decel);
  }
  return true;
}

Profile Profile::scaleDistance(float k) const {
  Profile p;
  if (!valid() || !std::isfinite(k) || k == 0.0f) return p;
  p.vpk_ = vpk_ * k;
  p.ta_  = ta_;
  p.tc_  = tc_;
  p.td_  = td_;
  return p;
}

Profile Profile::scaleTime(float k) const {
  Profile p;
  if (!valid() || !finitePos(k)) return p;
  p.vpk_ = vpk_ / k;
  p.ta_  = ta_ * k;
  p.tc_  = tc_ * k;
  p.td_  = td_ * k;
  return p;
}

// Both dumps are the same block of numbers with every rad multiplied by
// `scale` and relabelled: print() uses 1 rad/rad, printWithUnits() the wheel
// radius. The rpm figure is radius-independent, so it appears either way.
void Profile::printDump(float scale, const char* unit) const {
#ifdef ARDUINO
  if (!valid()) { Serial.print("Profile: <empty>"); Serial.println(); return; }
  // Serial.printf() lands in a vsnprintf() linked without float support, so
  // %f silently prints nothing. Use Serial.print(value, decimals) instead.
  Serial.print("Profile: ");     Serial.print(distance() * scale, 3);
  Serial.print(" ");             Serial.print(unit);
  Serial.print(" in ");          Serial.print(duration(), 3);
  Serial.println(" s");
  Serial.print("  peak vel   "); Serial.print(maxVelocity() * scale, 3);
  Serial.print(" ");             Serial.print(unit);
  Serial.print("/s (");
  Serial.print(maxVelocity() * 60.0f / (2.0f * PI_F), 1);
  Serial.print(" rpm) held ");
  Serial.print(cruiseTime(), 3);
  Serial.println(" s");
  Serial.print("  peak accel "); Serial.print(maxAccel() * scale, 3);
  Serial.print(" ");             Serial.print(unit);
  Serial.print("/s^2   peak decel "); Serial.print(maxDecel() * scale, 3);
  Serial.print(" ");             Serial.print(unit);
  Serial.println("/s^2");
  Serial.print("  accel      "); Serial.print(accelTime(), 3);
  Serial.print(" s   cruise "); Serial.print(cruiseTime(), 3);
  Serial.print(" s   decel ");  Serial.print(decelTime(), 3);
  Serial.println(" s");
#else
  if (!valid()) { std::printf("Profile: <empty>\n"); return; }
  std::printf("Profile: %.3f %s in %.3f s\n", (double)(distance() * scale), unit,
              (double)duration());
  std::printf("  peak vel   %.3f %s/s (%.1f rpm) held %.3f s\n",
              (double)(maxVelocity() * scale), unit,
              (double)(maxVelocity() * 60.0f / (2.0f * PI_F)), (double)cruiseTime());
  std::printf("  peak accel %.3f %s/s^2   peak decel %.3f %s/s^2\n",
              (double)(maxAccel() * scale), unit, (double)(maxDecel() * scale), unit);
  std::printf("  accel      %.3f s   cruise %.3f s   decel %.3f s\n",
              (double)accelTime(), (double)cruiseTime(), (double)decelTime());
#endif
}

void Profile::print() const { printDump(1.0f, "rad"); }

void Profile::printWithUnits(float wheel_radius, const char* unit) const {
  printDump(wheel_radius, unit ? unit : "");
}

}  // namespace rotev
