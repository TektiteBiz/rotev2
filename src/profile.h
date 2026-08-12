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

// One leg of a multi-leg move, in wheel radians. A sequence of these is handed
// to Profile::fromLegs(), which solves for the single cruise velocity that
// makes the whole sequence take a requested amount of time.
struct Leg {
  float dist;   // signed, rad
  float accel;  // magnitude, rad/s^2
  float decel;  // magnitude, rad/s^2
};

// Trapezoidal velocity move: constant acceleration up to the cruise speed,
// constant speed, then constant deceleration back to rest. Position is the
// integral of that, so it traces the S shape.
//   accel  (0 <= t < ta):        v = a*t,  a = vpk/ta
//   cruise (ta <= t < ta+tc):    v = vpk
//   decel  (last td seconds):    v ramps back to 0 at -d,  d = vpk/td
// The ramps may be asymmetric (a != d); the symmetric factories just pass the
// same rate twice, giving td == ta. A move that cannot reach vpk before it has
// to start stopping degenerates to a triangle (tc == 0) at the same rates.
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

  // Asymmetric complements to the two above: the ramp up runs at max_accel and
  // the ramp down at max_decel. Passing the same rate twice reproduces the
  // symmetric factory exactly.
  static Profile fromAccelDecel(float distance, float max_vel, float max_accel,
                                float max_decel);
  static Profile fromTimeAccelDecel(float distance, float time, float max_accel,
                                    float max_decel);

  // Fills out[0..n) with the profiles that cover `legs` back to back in exactly
  // `time` seconds, every one of them cruising at the same velocity so the
  // sequence reads as one continuous move rather than n unrelated ones. Each
  // leg keeps its own accel and decel; a leg too short to reach the shared
  // cruise velocity degenerates to a triangle and simply takes what time it
  // takes, with the rest of the sequence slowed to absorb the difference.
  //
  // Returns false if `time` is below the flat-out total (every leg triangular
  // at its own limits), in which case out[] holds that flat-out move -- still
  // runnable, just slower than asked for. `legs` and `out` are caller-owned
  // arrays of at least n entries; nothing is allocated. Degenerate inputs
  // leave every out[] entry empty and return false.
  static bool fromLegs(const Leg* legs, int n, float time, Profile* out);

  // Vertical stretch: same durations, distance/velocity/accel/decel all scale
  // by k, so the accel:decel ratio is preserved.
  Profile scaleDistance(float k) const;
  // Horizontal stretch: same distance, every duration scales by k, so velocity
  // scales by 1/k and both accel and decel by 1/k^2.
  Profile scaleTime(float k) const;

  // These are defined inline, not in profile.cpp, on purpose: at() is called
  // from the control ISR, which is __not_in_flash_func precisely so no XIP
  // cache miss can stretch a control tick. An out-of-line at() living in flash
  // would reintroduce exactly that stall, and inside the setpoint spinlock at
  // that. Inlining puts the arithmetic in the caller's RAM-resident code, and
  // trapezoidal segments need no transcendentals to get there.
  // signed, rad: half a ramp each side plus the full cruise
  float distance()   const { return vpk_ * (0.5f * (ta_ + td_) + tc_); }
  float duration()   const { return ta_ + tc_ + td_; }           // seconds
  float maxVelocity()const { return vpk_; }                      // signed, rad/s
  float maxAccel()   const { return ta_ > 0.0f ? vpk_ / ta_ : 0.0f; }
  // Signed like maxAccel(): the magnitude of the ramp down, whose actual
  // acceleration during the move is the negative of this.
  float maxDecel()   const { return td_ > 0.0f ? vpk_ / td_ : 0.0f; }
  float accelTime()  const { return ta_; }                       // seconds
  float cruiseTime() const { return tc_; }  // 0 for a triangular profile
  float decelTime()  const { return td_; }  // == accelTime() only if symmetric
  bool  symmetric()  const { return ta_ == td_; }
  bool  valid()      const { return ta_ > 0.0f && td_ > 0.0f && vpk_ != 0.0f; }

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

    const float a = vpk_ / ta_;              // signed ramp-up acceleration
    const float d = vpk_ / td_;              // signed ramp-down magnitude
    const float x_up = 0.5f * vpk_ * ta_;    // distance covered by the ramp up
    if (t >= T) {
      st.pos = distance();
    } else if (t < ta_) {
      st.vel  = a * t;
      st.pos  = 0.5f * a * t * t;
      st.acc  = a;
      st.done = false;
    } else if (t < ta_ + tc_) {
      st.pos  = x_up + vpk_ * (t - ta_);
      st.vel  = vpk_;
      st.done = false;
    } else {
      const float u = t - ta_ - tc_;
      st.vel  = vpk_ - d * u;
      st.pos  = x_up + vpk_ * tc_ + vpk_ * u - 0.5f * d * u * u;
      st.acc  = -d;
      st.done = false;
    }
    return st;
  }

  void print() const;  // human-readable dump to Serial, in rad

  // Same dump in linear units: every rad is multiplied by `wheel_radius` and
  // labelled `unit` (e.g. printWithUnits(0.016f, "m") for a 16 mm wheel).
  void printWithUnits(float wheel_radius, const char* unit) const;

 private:
  void printDump(float scale, const char* unit) const;

  // signed peak velocity, ramp-up time, cruise time, ramp-down time
  float vpk_, ta_, tc_, td_;
};

}  // namespace rotev
