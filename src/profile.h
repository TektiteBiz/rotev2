#pragma once

namespace rotev {

// Instantaneous state of a profile at some time.
struct ProfileState {
  float t;    // seconds since the profile started
  float pos;  // radians travelled (signed)
  float vel;  // rad/s (signed)
  float acc;  // rad/s^2 (signed)
  bool  done; // t >= duration()
};

// Trapezoidal velocity move: constant acceleration up to the cruise speed,
// constant speed, then constant deceleration back to rest. Position is the
// integral of that, so it traces the S shape.
//   accel  (0 <= t < ta):      v = a*t,  a = vpk/ta
//   cruise (ta <= t < ta+tc):  v = vpk
//   decel  (last ta seconds):  v ramps back to 0 at -a
// A move that cannot reach vpk before it has to start stopping degenerates to
// a triangle (tc == 0) at the same acceleration.
class Profile {
 public:
  Profile();  // empty: zero distance, zero duration, valid() == false

  // distance, peak velocity and peak acceleration. `distance` may be negative
  // (the move runs backwards); max_vel/max_accel are magnitudes.
  static Profile fromVelAccel(float distance, float max_vel, float max_accel);
  // distance covered in exactly `time` seconds without exceeding max_accel.
  // If `time` is too short for max_accel, the fastest possible profile at
  // max_accel is returned instead (duration() is then > time).
  static Profile fromTimeAccel(float distance, float time, float max_accel);

  // Vertical stretch: same durations, distance/velocity/accel all scale by k.
  Profile scaleDistance(float k) const;
  // Horizontal stretch: same distance, every duration scales by k, so
  // velocity scales by 1/k and acceleration by 1/k^2.
  Profile scaleTime(float k) const;

  // These are defined inline, not in profile.cpp, on purpose: at() is called
  // from the control ISR, which is __not_in_flash_func precisely so no XIP
  // cache miss can stretch a control tick. An out-of-line at() living in flash
  // would reintroduce exactly that stall, and inside the setpoint spinlock at
  // that. Inlining puts the arithmetic in the caller's RAM-resident code, and
  // trapezoidal segments need no transcendentals to get there.
  float distance()   const { return vpk_ * (ta_ + tc_); }        // signed, rad
  float duration()   const { return 2.0f * ta_ + tc_; }          // seconds
  float maxVelocity()const { return vpk_; }                      // signed, rad/s
  float maxAccel()   const { return ta_ > 0.0f ? vpk_ / ta_ : 0.0f; }
  float accelTime()  const { return ta_; }                       // seconds
  float cruiseTime() const { return tc_; }  // 0 for a triangular profile
  float decelTime()  const { return ta_; }  // == accelTime()
  bool  valid()      const { return ta_ > 0.0f && vpk_ != 0.0f; }

  // State at time t, clamped to [0, duration()].
  //
  // always_inline, not merely defined in-class: "inline" is a hint GCC
  // declined to take here, leaving an out-of-line copy in XIP flash that the
  // RAM-resident control ISR reached through a veneer -- a cache miss on that
  // path stretches a 24 kHz control tick by an unbounded amount. Forcing the
  // inline puts the arithmetic in the caller's own section instead. Verify
  // with: arm-none-eabi-objdump -d firmware.elf | grep Profile2at
  __attribute__((always_inline)) inline ProfileState at(float t) const {
    ProfileState st{0.0f, 0.0f, 0.0f, 0.0f, true};
    if (!valid()) return st;

    const float T = duration();
    if (!(t > 0.0f)) t = 0.0f;  // also catches NaN
    if (t > T) t = T;
    st.t = t;

    const float a = vpk_ / ta_;              // signed ramp acceleration
    const float x_ramp = 0.5f * vpk_ * ta_;  // distance covered by one ramp
    if (t >= T) {
      st.pos = distance();
    } else if (t < ta_) {
      st.vel  = a * t;
      st.pos  = 0.5f * a * t * t;
      st.acc  = a;
      st.done = false;
    } else if (t < ta_ + tc_) {
      st.pos  = x_ramp + vpk_ * (t - ta_);
      st.vel  = vpk_;
      st.done = false;
    } else {
      const float u = t - ta_ - tc_;
      st.vel  = vpk_ - a * u;
      st.pos  = x_ramp + vpk_ * tc_ + vpk_ * u - 0.5f * a * u * u;
      st.acc  = -a;
      st.done = false;
    }
    return st;
  }

  void print() const;  // human-readable dump to Serial

 private:
  float vpk_, ta_, tc_;  // signed peak velocity, ramp time, cruise time
};

}  // namespace rotev
