#include <unity.h>
#include <cmath>
#include "profile.h"
using namespace rotev;

void setUp() {} void tearDown() {}

void test_default_profile_is_empty() {
  Profile p;
  TEST_ASSERT_FALSE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.maxAccel());
}

void test_trapezoidal_branch_reaches_max_vel() {
  // ramps need V*ta = 10 * (10/5) = 20 rad, far less than 1000.
  Profile p = Profile::fromVelAccel(1000.0f, 10.0f, 5.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.0f, p.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 5.0f, p.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.0f / 5.0f, p.accelTime());
  TEST_ASSERT_TRUE(p.cruiseTime() > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 1000.0f, p.distance());
}

void test_triangular_branch_never_reaches_max_vel() {
  // 1 rad at 100 rad/s would need 1000 rad of ramp: peak velocity is capped.
  Profile p = Profile::fromVelAccel(1.0f, 100.0f, 10.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.cruiseTime());
  TEST_ASSERT_TRUE(p.maxVelocity() < 100.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, std::sqrt(10.0f * 1.0f), p.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 10.0f, p.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0f, p.distance());
}

void test_accessors_are_self_consistent() {
  Profile p = Profile::fromVelAccel(250.0f, 20.0f, 8.0f);
  TEST_ASSERT_TRUE(p.symmetric());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.accelTime(), p.decelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, p.maxAccel(), p.maxDecel());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0f * p.accelTime() + p.cruiseTime(), p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, p.maxVelocity() * (p.accelTime() + p.cruiseTime()), p.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, p.maxVelocity() / p.accelTime(), p.maxAccel());
}

void test_from_time_accel_reproduces_requested_duration() {
  // minimum time at A=10 for D=100 is 2*sqrt(D/A) = 6.325 s, so 12 s is reachable.
  Profile p = Profile::fromTimeAccel(100.0f, 12.0f, 10.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 12.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 100.0f, p.distance());
  TEST_ASSERT_TRUE(p.cruiseTime() > 0.0f);          // smaller root always cruises
  TEST_ASSERT_TRUE(std::fabs(p.maxAccel()) <= 10.0f + 1e-3f);
}

void test_from_time_accel_falls_back_when_time_unreachable() {
  Profile p = Profile::fromTimeAccel(100.0f, 1.0f, 10.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_TRUE(p.duration() > 1.0f);
  // The fallback is the time-optimal triangular move at max_accel.
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.0f * std::sqrt(100.0f / 10.0f), p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 100.0f, p.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 10.0f, p.maxAccel());
}

void test_from_time_accel_survives_time_far_above_minimum() {
  // Regression: v = A*(T-sqrt(T*T-4D/A))/2 cancels catastrophically once T*T
  // swamps 4D/A. Here Tmin = 2*sqrt(D/A) = 0.063 s, so T=300 s is 4700x above
  // it -- deeply reachable -- yet the old form returned an EMPTY profile.
  Profile p = Profile::fromTimeAccel(0.1f, 300.0f, 100.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 300.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.1f, p.distance());
  TEST_ASSERT_TRUE(p.cruiseTime() > 0.0f);
}

void test_from_time_accel_holds_requested_time_across_a_wide_sweep() {
  // One revolution slow-indexed at phase3's accel limit, and friends: every
  // one of these is far above Tmin, so "exactly `time` seconds" must hold.
  const float D[] = {0.01f, 0.5f, 6.2832f, 6.2832f};
  const float T[] = {60.0f, 200.0f, 600.0f, 300.0f};
  const float A[] = {100.0f, 100.0f, 89.0f, 50.0f};
  for (int k = 0; k < 4; ++k) {
    Profile p = Profile::fromTimeAccel(D[k], T[k], A[k]);
    TEST_ASSERT_TRUE(p.valid());
    TEST_ASSERT_FLOAT_WITHIN(T[k] * 1e-3f, T[k], p.duration());
    TEST_ASSERT_FLOAT_WITHIN(D[k] * 1e-3f, D[k], p.distance());
    TEST_ASSERT_TRUE(std::fabs(p.maxAccel()) <= A[k] + 1e-3f);
  }
}

void test_from_time_accel_at_exactly_minimum_time() {
  float tmin = 2.0f * std::sqrt(100.0f / 10.0f);  // triangular, disc == 0
  Profile p = Profile::fromTimeAccel(100.0f, tmin, 10.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, tmin, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 100.0f, p.distance());
}

void test_scale_distance_keeps_durations_and_scales_the_rest() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  Profile q = p.scaleDistance(2.5f);
  TEST_ASSERT_TRUE(q.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.duration(), q.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.accelTime(), q.accelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.cruiseTime(), q.cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 2.5f * p.distance(), q.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.5f * p.maxVelocity(), q.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.5f * p.maxAccel(), q.maxAccel());
}

void test_scale_distance_negative_reverses_the_move() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  Profile q = p.scaleDistance(-1.0f);
  TEST_ASSERT_TRUE(q.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.duration(), q.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -p.distance(), q.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -p.maxVelocity(), q.maxVelocity());
}

void test_scale_time_keeps_distance() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  Profile q = p.scaleTime(2.0f);
  TEST_ASSERT_TRUE(q.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, p.distance(), q.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0f * p.duration(), q.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0f * p.accelTime(), q.accelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0f * p.cruiseTime(), q.cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, p.maxVelocity() / 2.0f, q.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, p.maxAccel() / 4.0f, q.maxAccel());
}

void test_scale_time_rejects_non_positive_k() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  TEST_ASSERT_FALSE(p.scaleTime(0.0f).valid());
  TEST_ASSERT_FALSE(p.scaleTime(-2.0f).valid());
}

void test_at_start_is_at_rest() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  ProfileState s = p.at(0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.t);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, s.pos);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, s.vel);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, p.maxAccel(), s.acc);  // full accel from t=0
  TEST_ASSERT_FALSE(s.done);
}

void test_at_cruise_midpoint_holds_peak_velocity() {
  Profile p = Profile::fromVelAccel(1000.0f, 10.0f, 5.0f);
  ProfileState s = p.at(p.duration() * 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, p.maxVelocity(), s.vel);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, s.acc);
  TEST_ASSERT_FLOAT_WITHIN(1e-1, p.distance() * 0.5f, s.pos);  // symmetric profile
  TEST_ASSERT_FALSE(s.done);
}

void test_at_ramp_midpoint_hits_peak_accel() {
  Profile p = Profile::fromVelAccel(1000.0f, 10.0f, 5.0f);
  ProfileState s = p.at(p.accelTime() * 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, p.maxAccel(), s.acc);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, p.maxVelocity() * 0.5f, s.vel);
}

void test_at_end_and_past_end_hold_the_final_position() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  ProfileState e = p.at(p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, p.distance(), e.pos);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, e.vel);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, e.acc);
  TEST_ASSERT_TRUE(e.done);

  ProfileState past = p.at(p.duration() * 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.duration(), past.t);  // clamped
  TEST_ASSERT_FLOAT_WITHIN(1e-2, p.distance(), past.pos);
  TEST_ASSERT_TRUE(past.done);
}

void test_at_negative_time_clamps_to_start() {
  Profile p = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  ProfileState s = p.at(-5.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.t);
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.pos);
}

void test_at_position_is_monotonic_and_matches_phase_boundaries() {
  Profile p = Profile::fromVelAccel(1000.0f, 10.0f, 5.0f);
  float prev = -1.0f;
  for (int i = 0; i <= 2000; ++i) {
    ProfileState s = p.at(p.duration() * (float)i / 2000.0f);
    TEST_ASSERT_TRUE(s.pos >= prev - 1e-3f);
    prev = s.pos;
  }
  // Each ramp covers vpk*ta/2; the phase pieces must join without a step.
  ProfileState a = p.at(p.accelTime() - 1e-4f), b = p.at(p.accelTime() + 1e-4f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, a.pos, b.pos);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, a.vel, b.vel);
}

// Integrates at().vel with the trapezoid rule; the result must be distance().
static double integrateVel(const Profile& p, int n) {
  double T = p.duration(), h = T / n, sum = 0.0;
  for (int i = 0; i <= n; ++i) {
    double w = (i == 0 || i == n) ? 0.5 : 1.0;
    sum += w * (double)p.at((float)(h * i)).vel;
  }
  return sum * h;
}

void test_integrated_velocity_matches_distance_trapezoidal() {
  Profile p = Profile::fromVelAccel(1000.0f, 10.0f, 5.0f);
  double d = integrateVel(p, 200000);
  TEST_ASSERT_TRUE(std::fabs(d - (double)p.distance()) < 1e-2);
}

void test_integrated_velocity_matches_distance_triangular() {
  Profile p = Profile::fromVelAccel(1.0f, 100.0f, 10.0f);
  double d = integrateVel(p, 200000);
  TEST_ASSERT_TRUE(std::fabs(d - (double)p.distance()) < 1e-5);
}

void test_negative_distance_mirrors_the_positive_move() {
  Profile fwd = Profile::fromVelAccel(100.0f, 10.0f, 4.0f);
  Profile rev = Profile::fromVelAccel(-100.0f, 10.0f, 4.0f);
  TEST_ASSERT_TRUE(rev.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, fwd.duration(), rev.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -100.0f, rev.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -fwd.maxVelocity(), rev.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -fwd.maxAccel(), rev.maxAccel());
  ProfileState s = rev.at(rev.duration() * 0.5f);
  TEST_ASSERT_TRUE(s.vel < 0.0f && s.pos < 0.0f);
}

void test_negative_distance_from_time_accel() {
  Profile p = Profile::fromTimeAccel(-100.0f, 12.0f, 10.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 12.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -100.0f, p.distance());
}

void test_degenerate_inputs_return_empty_profiles() {
  const Profile bad[] = {
    Profile::fromVelAccel(0.0f, 10.0f, 4.0f),
    Profile::fromVelAccel(100.0f, 0.0f, 4.0f),
    Profile::fromVelAccel(100.0f, 10.0f, 0.0f),
    Profile::fromVelAccel(NAN, 10.0f, 4.0f),
    Profile::fromVelAccel(100.0f, INFINITY, 4.0f),
    Profile::fromVelAccel(INFINITY, 10.0f, 4.0f),
    Profile::fromTimeAccel(0.0f, 12.0f, 10.0f),
    Profile::fromTimeAccel(100.0f, 0.0f, 10.0f),
    Profile::fromTimeAccel(100.0f, -3.0f, 10.0f),
    Profile::fromTimeAccel(100.0f, 12.0f, 0.0f),
    Profile::fromTimeAccel(100.0f, NAN, 10.0f),
    Profile::fromTimeAccel(NAN, 12.0f, 10.0f),
  };
  for (const Profile& p : bad) {
    TEST_ASSERT_FALSE(p.valid());
    TEST_ASSERT_FALSE(std::isnan(p.distance()));
    TEST_ASSERT_FALSE(std::isnan(p.duration()));
    TEST_ASSERT_FALSE(std::isnan(p.maxAccel()));
    ProfileState s = p.at(1.0f);
    TEST_ASSERT_TRUE(s.done);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.pos);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.vel);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.acc);
  }
}

void test_scaling_an_empty_profile_stays_empty() {
  Profile empty;
  TEST_ASSERT_FALSE(empty.scaleDistance(2.0f).valid());
  TEST_ASSERT_FALSE(empty.scaleTime(2.0f).valid());
  // k == 0 collapses the move to nothing, which is not a runnable profile.
  TEST_ASSERT_FALSE(Profile::fromVelAccel(100.0f, 10.0f, 4.0f).scaleDistance(0.0f).valid());
}

// --- asymmetric ramps -------------------------------------------------------

void test_accel_decel_trapezoid_honours_both_rates() {
  // ramps cover V^2/2*(1/A+1/D) = 100/2*(1/5+1/20) = 12.5 rad, well under 500.
  Profile p = Profile::fromAccelDecel(500.0f, 10.0f, 5.0f, 20.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FALSE(p.symmetric());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.0f, p.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 5.0f, p.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 20.0f, p.maxDecel());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.0f / 5.0f, p.accelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10.0f / 20.0f, p.decelTime());
  TEST_ASSERT_TRUE(p.cruiseTime() > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 500.0f, p.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, p.accelTime() + p.cruiseTime() + p.decelTime(),
                           p.duration());
}

void test_accel_decel_triangle_splits_by_rate_ratio() {
  // 1 rad is far too short to reach 100 rad/s: k*vpk^2 == D with k = (1/2+1/8)/2.
  const float k = 0.5f * (1.0f / 2.0f + 1.0f / 8.0f);
  Profile p = Profile::fromAccelDecel(1.0f, 100.0f, 2.0f, 8.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, std::sqrt(1.0f / k), p.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.0f, p.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 8.0f, p.maxDecel());
  // the harder ramp takes a quarter the time of the gentler one
  TEST_ASSERT_FLOAT_WITHIN(1e-4, p.accelTime() / 4.0f, p.decelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.0f, p.distance());
}

void test_accel_decel_equal_rates_match_the_symmetric_factory() {
  const float D[] = {1000.0f, 1.0f, -250.0f};
  for (int i = 0; i < 3; ++i) {
    Profile a = Profile::fromVelAccel(D[i], 10.0f, 5.0f);
    Profile b = Profile::fromAccelDecel(D[i], 10.0f, 5.0f, 5.0f);
    TEST_ASSERT_TRUE(b.valid());
    TEST_ASSERT_FLOAT_WITHIN(1e-5, a.maxVelocity(), b.maxVelocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-5, a.accelTime(), b.accelTime());
    TEST_ASSERT_FLOAT_WITHIN(1e-5, a.cruiseTime(), b.cruiseTime());
    TEST_ASSERT_FLOAT_WITHIN(1e-5, a.decelTime(), b.decelTime());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, a.distance(), b.distance());
  }
}

void test_accel_decel_at_matches_the_two_ramp_rates() {
  Profile p = Profile::fromAccelDecel(500.0f, 10.0f, 5.0f, 20.0f);
  ProfileState up = p.at(0.5f * p.accelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 5.0f, up.acc);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f * p.maxVelocity(), up.vel);
  ProfileState cruise = p.at(p.accelTime() + 0.5f * p.cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 10.0f, cruise.vel);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, cruise.acc);
  ProfileState down = p.at(p.accelTime() + p.cruiseTime() + 0.5f * p.decelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -20.0f, down.acc);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.5f * p.maxVelocity(), down.vel);
  ProfileState end = p.at(p.duration());
  TEST_ASSERT_TRUE(end.done);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, end.vel);
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 500.0f, end.pos);
}

void test_integrated_velocity_matches_distance_asymmetric() {
  // trapezoid and triangle, both with a 4:1 rate split
  const Profile ps[] = {Profile::fromAccelDecel(500.0f, 10.0f, 5.0f, 20.0f),
                        Profile::fromAccelDecel(1.0f, 100.0f, 2.0f, 8.0f)};
  for (const Profile& p : ps) {
    const int N = 20000;
    const float dt = p.duration() / N;
    float x = 0.0f;
    for (int i = 0; i < N; ++i) x += p.at((i + 0.5f) * dt).vel * dt;
    TEST_ASSERT_FLOAT_WITHIN(std::fabs(p.distance()) * 1e-3f, p.distance(), x);
  }
}

void test_time_accel_decel_reproduces_requested_duration() {
  // Tmin at these rates is 2*sqrt(k*D) with k = (1/10+1/40)/2 = 0.0625, so
  // Tmin = 5 s for D = 100: 12 s is comfortably reachable.
  Profile p = Profile::fromTimeAccelDecel(100.0f, 12.0f, 10.0f, 40.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 12.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 100.0f, p.distance());
  TEST_ASSERT_TRUE(p.cruiseTime() > 0.0f);
  TEST_ASSERT_TRUE(std::fabs(p.maxAccel()) <= 10.0f + 1e-3f);
  TEST_ASSERT_TRUE(std::fabs(p.maxDecel()) <= 40.0f + 1e-3f);
}

void test_time_accel_decel_falls_back_when_time_unreachable() {
  const float k = 0.5f * (1.0f / 10.0f + 1.0f / 40.0f);
  Profile p = Profile::fromTimeAccelDecel(100.0f, 1.0f, 10.0f, 40.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 2.0f * std::sqrt(k * 100.0f), p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, p.cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 100.0f, p.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 10.0f, p.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 40.0f, p.maxDecel());
}

void test_time_accel_decel_equal_rates_match_the_symmetric_factory() {
  Profile a = Profile::fromTimeAccel(100.0f, 12.0f, 10.0f);
  Profile b = Profile::fromTimeAccelDecel(100.0f, 12.0f, 10.0f, 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, a.maxVelocity(), b.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, a.duration(), b.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, a.distance(), b.distance());
}

void test_time_accel_decel_survives_time_far_above_minimum() {
  Profile p = Profile::fromTimeAccelDecel(0.1f, 300.0f, 100.0f, 25.0f);
  TEST_ASSERT_TRUE(p.valid());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 300.0f, p.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.1f, p.distance());
  TEST_ASSERT_TRUE(p.cruiseTime() > 0.0f);
}

void test_negative_distance_mirrors_an_asymmetric_move() {
  Profile f = Profile::fromAccelDecel(500.0f, 10.0f, 5.0f, 20.0f);
  Profile r = Profile::fromAccelDecel(-500.0f, 10.0f, 5.0f, 20.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, f.duration(), r.duration());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, f.accelTime(), r.accelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-5, f.decelTime(), r.decelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -f.distance(), r.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -f.maxVelocity(), r.maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -f.maxAccel(), r.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, -f.maxDecel(), r.maxDecel());
}

void test_scaling_preserves_the_accel_decel_ratio() {
  Profile p = Profile::fromAccelDecel(500.0f, 10.0f, 5.0f, 20.0f);
  const float ratio = p.maxDecel() / p.maxAccel();

  Profile d = p.scaleDistance(2.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, p.decelTime(), d.decelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.5f * p.maxAccel(), d.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 2.5f * p.maxDecel(), d.maxDecel());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, ratio, d.maxDecel() / d.maxAccel());

  Profile t = p.scaleTime(2.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 2.0f * p.decelTime(), t.decelTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, p.distance(), t.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, p.maxAccel() / 4.0f, t.maxAccel());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, p.maxDecel() / 4.0f, t.maxDecel());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, ratio, t.maxDecel() / t.maxAccel());
}

void test_degenerate_asymmetric_inputs_return_empty_profiles() {
  const Profile bad[] = {
    Profile::fromAccelDecel(100.0f, 10.0f, 4.0f, 0.0f),
    Profile::fromAccelDecel(100.0f, 10.0f, 0.0f, 4.0f),
    Profile::fromAccelDecel(100.0f, 10.0f, 4.0f, NAN),
    Profile::fromAccelDecel(100.0f, 10.0f, 4.0f, INFINITY),
    Profile::fromAccelDecel(0.0f, 10.0f, 4.0f, 8.0f),
    Profile::fromTimeAccelDecel(100.0f, 12.0f, 10.0f, 0.0f),
    Profile::fromTimeAccelDecel(100.0f, 12.0f, 10.0f, NAN),
    Profile::fromTimeAccelDecel(100.0f, -3.0f, 10.0f, 40.0f),
  };
  for (const Profile& p : bad) {
    TEST_ASSERT_FALSE(p.valid());
    TEST_ASSERT_FALSE(std::isnan(p.distance()));
    TEST_ASSERT_FALSE(std::isnan(p.duration()));
    TEST_ASSERT_FALSE(std::isnan(p.maxDecel()));
    ProfileState s = p.at(1.0f);
    TEST_ASSERT_TRUE(s.done);
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, s.vel);
  }
}

// --- multi-leg scheduling ---------------------------------------------------

static float legsTotal(const Profile* p, int n) {
  float t = 0.0f;
  for (int i = 0; i < n; ++i) t += p[i].duration();
  return t;
}

void test_from_legs_hits_the_budget_with_no_leg_saturated() {
  // Long legs at generous rates: every one cruises, so the closed form is exact.
  const Leg legs[] = {{30.0f, 5.0f, 5.0f}, {70.0f, 5.0f, 20.0f}, {50.0f, 10.0f, 10.0f}};
  Profile out[3];
  TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, 40.0f, out));
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 40.0f, legsTotal(out, 3));
  for (int i = 0; i < 3; ++i) {
    TEST_ASSERT_TRUE(out[i].valid());
    TEST_ASSERT_FLOAT_WITHIN(std::fabs(legs[i].dist) * 1e-3f, legs[i].dist, out[i].distance());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, legs[i].accel, out[i].maxAccel());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, legs[i].decel, out[i].maxDecel());
    TEST_ASSERT_TRUE(out[i].cruiseTime() > 0.0f);           // nothing saturated
    TEST_ASSERT_FLOAT_WITHIN(1e-3, out[0].maxVelocity(), out[i].maxVelocity());
  }
}

void test_from_legs_absorbs_a_saturated_leg() {
  // The 0.5 rad leg is too short to reach the shared cruise speed (its ramps
  // meet at 1.13 rad/s, below the 1.35 the other two settle on), so it pins
  // triangular and they must stretch to hold the total. This is the case the
  // naive quadratic gets wrong -- it credits this leg with time it never spends
  // and the sequence lands a few ms short.
  const Leg legs[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}, {0.5f, 1.75f, 4.78f}};
  Profile out[3];
  TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, 9.9f, out));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 9.9f, legsTotal(out, 3));
  for (int i = 0; i < 3; ++i)
    TEST_ASSERT_FLOAT_WITHIN(std::fabs(legs[i].dist) * 1e-3f, legs[i].dist, out[i].distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, out[2].cruiseTime());        // pinned triangular
  TEST_ASSERT_TRUE(out[2].maxVelocity() < out[1].maxVelocity());    // below the shared speed
  TEST_ASSERT_TRUE(out[0].cruiseTime() > 0.0f);
  TEST_ASSERT_TRUE(out[1].cruiseTime() > 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, out[0].maxVelocity(), out[1].maxVelocity());
}

void test_from_legs_absorbs_a_saturating_first_leg() {
  // A 0.05 rad leg saturates even at the lower bracket, exercising the halving
  // loop that walks `lo` down until it is genuinely below the root.
  const Leg legs[] = {{0.05f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}, {3.0f, 1.75f, 4.78f}};
  Profile out[3];
  TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, 9.9f, out));
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 9.9f, legsTotal(out, 3));
  TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, out[0].cruiseTime());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.05f, out[0].distance());
}

void test_from_legs_holds_the_budget_across_a_sweep() {
  // t_min for this set is 7.74 s, so every budget below is reachable.
  const Leg legs[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}, {0.5f, 1.75f, 4.78f}};
  const float T[] = {8.0f, 9.9f, 15.0f, 60.0f, 600.0f};
  for (int k = 0; k < 5; ++k) {
    Profile out[3];
    TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, T[k], out));
    TEST_ASSERT_FLOAT_WITHIN(T[k] * 1e-3f, T[k], legsTotal(out, 3));
    for (int i = 0; i < 3; ++i) {
      TEST_ASSERT_TRUE(out[i].valid());
      TEST_ASSERT_FLOAT_WITHIN(std::fabs(legs[i].dist) * 1e-3f, legs[i].dist, out[i].distance());
      TEST_ASSERT_TRUE(std::fabs(out[i].maxAccel()) <= legs[i].accel + 1e-3f);
      TEST_ASSERT_TRUE(std::fabs(out[i].maxDecel()) <= legs[i].decel + 1e-3f);
    }
  }
}

void test_from_legs_total_is_monotone_in_the_budget() {
  // The solver inverts a monotone non-increasing T_total(v), so a longer budget
  // can never come back with a faster sequence.
  const Leg legs[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}, {0.5f, 1.75f, 4.78f}};
  float prev_v = 1e30f;
  for (float T = 8.0f; T <= 40.0f; T += 0.25f) {
    Profile out[3];
    TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, T, out));
    const float v = out[1].maxVelocity();
    TEST_ASSERT_TRUE(v <= prev_v + 1e-4f);
    prev_v = v;
  }
}

void test_from_legs_reports_an_impossible_budget_flat_out() {
  const Leg legs[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}, {0.5f, 1.75f, 4.78f}};
  Profile out[3];
  TEST_ASSERT_FALSE(Profile::fromLegs(legs, 3, 5.0f, out));
  // out[] still holds a runnable move: every leg triangular at its own limits.
  float flat_out = 0.0f;
  for (int i = 0; i < 3; ++i) {
    TEST_ASSERT_TRUE(out[i].valid());
    TEST_ASSERT_FLOAT_WITHIN(std::fabs(legs[i].dist) * 1e-3f, legs[i].dist, out[i].distance());
    TEST_ASSERT_FLOAT_WITHIN(1e-6, 0.0f, out[i].cruiseTime());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, legs[i].accel, out[i].maxAccel());
    TEST_ASSERT_FLOAT_WITHIN(1e-3, legs[i].decel, out[i].maxDecel());
    flat_out += out[i].duration();
  }
  TEST_ASSERT_TRUE(flat_out > 5.0f);
  // and nothing shorter than that is achievable
  Profile probe[3];
  TEST_ASSERT_FALSE(Profile::fromLegs(legs, 3, flat_out * 0.999f, probe));
  TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, flat_out * 1.01f, probe));
}

void test_from_legs_single_leg_matches_from_time_accel_decel() {
  const Leg legs[] = {{100.0f, 10.0f, 40.0f}};
  Profile out[1];
  TEST_ASSERT_TRUE(Profile::fromLegs(legs, 1, 12.0f, out));
  Profile ref = Profile::fromTimeAccelDecel(100.0f, 12.0f, 10.0f, 40.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, ref.maxVelocity(), out[0].maxVelocity());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 12.0f, out[0].duration());
}

void test_from_legs_handles_negative_and_mixed_directions() {
  const Leg legs[] = {{3.0f, 1.75f, 4.78f}, {-7.0f, 1.0f, 1.75f}, {2.0f, 1.75f, 4.78f}};
  Profile out[3];
  TEST_ASSERT_TRUE(Profile::fromLegs(legs, 3, 12.0f, out));
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 12.0f, legsTotal(out, 3));
  TEST_ASSERT_FLOAT_WITHIN(1e-2, 3.0f, out[0].distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-2, -7.0f, out[1].distance());
  TEST_ASSERT_TRUE(out[1].maxVelocity() < 0.0f);
  // the reversed leg cruises at the same speed, just the other way
  TEST_ASSERT_FLOAT_WITHIN(1e-3, out[0].maxVelocity(), -out[1].maxVelocity());
}

void test_from_legs_rejects_degenerate_input() {
  const Leg ok[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}};
  const Leg bad_dist[]  = {{3.0f, 1.75f, 4.78f}, {0.0f, 1.0f, 1.75f}};
  const Leg bad_accel[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 0.0f, 1.75f}};
  const Leg bad_decel[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, NAN}};
  const Leg bad_inf[]   = {{INFINITY, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}};
  Profile out[2];

  const Leg* bad[] = {bad_dist, bad_accel, bad_decel, bad_inf};
  for (int k = 0; k < 4; ++k) {
    TEST_ASSERT_FALSE(Profile::fromLegs(bad[k], 2, 20.0f, out));
    for (int i = 0; i < 2; ++i) TEST_ASSERT_FALSE(out[i].valid());
  }
  TEST_ASSERT_FALSE(Profile::fromLegs(ok, 0, 20.0f, out));
  TEST_ASSERT_FALSE(Profile::fromLegs(ok, -1, 20.0f, out));
  TEST_ASSERT_FALSE(Profile::fromLegs(nullptr, 2, 20.0f, out));
  TEST_ASSERT_FALSE(Profile::fromLegs(ok, 2, 20.0f, nullptr));
  TEST_ASSERT_FALSE(Profile::fromLegs(ok, 2, NAN, out));
  TEST_ASSERT_FALSE(Profile::fromLegs(ok, 2, -5.0f, out));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_default_profile_is_empty);
  RUN_TEST(test_trapezoidal_branch_reaches_max_vel);
  RUN_TEST(test_triangular_branch_never_reaches_max_vel);
  RUN_TEST(test_accessors_are_self_consistent);
  RUN_TEST(test_from_time_accel_reproduces_requested_duration);
  RUN_TEST(test_from_time_accel_falls_back_when_time_unreachable);
  RUN_TEST(test_from_time_accel_survives_time_far_above_minimum);
  RUN_TEST(test_from_time_accel_holds_requested_time_across_a_wide_sweep);
  RUN_TEST(test_from_time_accel_at_exactly_minimum_time);
  RUN_TEST(test_scale_distance_keeps_durations_and_scales_the_rest);
  RUN_TEST(test_scale_distance_negative_reverses_the_move);
  RUN_TEST(test_scale_time_keeps_distance);
  RUN_TEST(test_scale_time_rejects_non_positive_k);
  RUN_TEST(test_at_start_is_at_rest);
  RUN_TEST(test_at_cruise_midpoint_holds_peak_velocity);
  RUN_TEST(test_at_ramp_midpoint_hits_peak_accel);
  RUN_TEST(test_at_end_and_past_end_hold_the_final_position);
  RUN_TEST(test_at_negative_time_clamps_to_start);
  RUN_TEST(test_at_position_is_monotonic_and_matches_phase_boundaries);
  RUN_TEST(test_integrated_velocity_matches_distance_trapezoidal);
  RUN_TEST(test_integrated_velocity_matches_distance_triangular);
  RUN_TEST(test_negative_distance_mirrors_the_positive_move);
  RUN_TEST(test_negative_distance_from_time_accel);
  RUN_TEST(test_degenerate_inputs_return_empty_profiles);
  RUN_TEST(test_scaling_an_empty_profile_stays_empty);
  RUN_TEST(test_accel_decel_trapezoid_honours_both_rates);
  RUN_TEST(test_accel_decel_triangle_splits_by_rate_ratio);
  RUN_TEST(test_accel_decel_equal_rates_match_the_symmetric_factory);
  RUN_TEST(test_accel_decel_at_matches_the_two_ramp_rates);
  RUN_TEST(test_integrated_velocity_matches_distance_asymmetric);
  RUN_TEST(test_time_accel_decel_reproduces_requested_duration);
  RUN_TEST(test_time_accel_decel_falls_back_when_time_unreachable);
  RUN_TEST(test_time_accel_decel_equal_rates_match_the_symmetric_factory);
  RUN_TEST(test_time_accel_decel_survives_time_far_above_minimum);
  RUN_TEST(test_negative_distance_mirrors_an_asymmetric_move);
  RUN_TEST(test_scaling_preserves_the_accel_decel_ratio);
  RUN_TEST(test_degenerate_asymmetric_inputs_return_empty_profiles);
  RUN_TEST(test_from_legs_hits_the_budget_with_no_leg_saturated);
  RUN_TEST(test_from_legs_absorbs_a_saturated_leg);
  RUN_TEST(test_from_legs_absorbs_a_saturating_first_leg);
  RUN_TEST(test_from_legs_holds_the_budget_across_a_sweep);
  RUN_TEST(test_from_legs_total_is_monotone_in_the_budget);
  RUN_TEST(test_from_legs_reports_an_impossible_budget_flat_out);
  RUN_TEST(test_from_legs_single_leg_matches_from_time_accel_decel);
  RUN_TEST(test_from_legs_handles_negative_and_mixed_directions);
  RUN_TEST(test_from_legs_rejects_degenerate_input);
  return UNITY_END();
}
