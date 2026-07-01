#include <unity.h>
#include <cmath>
#include "foc_math.h"
using namespace rotev;

void setUp() {} void tearDown() {}

void test_park_zero_angle_is_identity() {
  DQ dq = park({1.0f, 0.5f}, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, dq.d);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, dq.q);
}
void test_park_ninety_deg_rotates() {
  // theta=pi/2: d = a*cos+b*sin = b ; q = -a*sin+b*cos = -a
  DQ dq = park({1.0f, 0.0f}, (float)M_PI/2);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, dq.d);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, -1.0f, dq.q);
}
void test_inverse_park_zero_angle_normalizes_by_vbus() {
  AB ab = inversePark(6.0f, 0.0f, 0.0f, 12.0f); // ud=6V of 12V -> 0.5
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, ab.a);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, ab.b);
}
void test_inverse_park_clamps_to_unit() {
  AB ab = inversePark(24.0f, 0.0f, 0.0f, 12.0f); // 2.0 -> clamp 1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, ab.a);
}

void test_pi_proportional_only_first_step() {
  PIState s; piReset(s);
  // dt small so integral term tiny; kp=3.5, error=1 -> ~3.5 + ki*dt
  float out = piStep(s, 1.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 1000.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.5f + 3500.0f*(1.0f/24000.0f), out);
}
void test_pi_output_clamped_to_limit() {
  PIState s; piReset(s);
  float out = piStep(s, 100.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 5.0f, out);
}
void test_pi_antiwindup_stops_integrating_when_saturated() {
  PIState s; piReset(s);
  for (int i = 0; i < 1000; ++i) piStep(s, 100.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 5.0f);
  // After saturation, a single negative error should immediately reduce output
  float out = piStep(s, -100.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 5.0f);
  TEST_ASSERT_TRUE(out < 5.0f);
}
void test_pi_reset_clears_integrator() {
  PIState s; piReset(s);
  piStep(s, 1.0f, 3.5f, 3500.0f, 0.1f, 1000.0f);
  piReset(s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integ);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_park_zero_angle_is_identity);
  RUN_TEST(test_park_ninety_deg_rotates);
  RUN_TEST(test_inverse_park_zero_angle_normalizes_by_vbus);
  RUN_TEST(test_inverse_park_clamps_to_unit);
  RUN_TEST(test_pi_proportional_only_first_step);
  RUN_TEST(test_pi_output_clamped_to_limit);
  RUN_TEST(test_pi_antiwindup_stops_integrating_when_saturated);
  RUN_TEST(test_pi_reset_clears_integrator);
  return UNITY_END();
}
