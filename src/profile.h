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

// Jerk-free S-curve move: the velocity ramps up as a raised cosine, cruises,
// then ramps down the same way, so acceleration starts and ends at zero.
//   accel phase (0 <= t < ta):   v = vpk/2 * (1 - cos(pi*t/ta))
//   cruise      (ta <= t < ta+tc): v = vpk
//   decel phase (mirror of accel)
// Peak acceleration is pi*vpk/(2*ta), reached at the middle of each ramp.
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

  float distance()   const;  // signed, radians
  float duration()   const;  // seconds, accel + cruise + decel
  float maxVelocity()const;  // signed peak (cruise) velocity, rad/s
  float maxAccel()   const;  // signed peak acceleration, rad/s^2
  float accelTime()  const;  // seconds
  float cruiseTime() const;  // seconds (0 for a triangular profile)
  float decelTime()  const;  // seconds, == accelTime()
  bool  valid()      const;  // false for an empty/degenerate profile

  ProfileState at(float t) const;  // clamped to [0, duration()]
  void print() const;              // human-readable dump to Serial

 private:
  float vpk_, ta_, tc_;  // signed peak velocity, ramp time, cruise time
};

}  // namespace rotev
