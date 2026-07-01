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

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_park_zero_angle_is_identity);
  RUN_TEST(test_park_ninety_deg_rotates);
  RUN_TEST(test_inverse_park_zero_angle_normalizes_by_vbus);
  RUN_TEST(test_inverse_park_clamps_to_unit);
  return UNITY_END();
}
