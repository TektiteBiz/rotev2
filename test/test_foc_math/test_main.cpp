#include <unity.h>
#include "constants.h"
using namespace rotev;
void setUp() {}
void tearDown() {}
void test_derived_gains() {
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 3.5f, KP);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 3500.0f, KI);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 1.5f, VOLTS_PER_AMP);
}
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_derived_gains);
  return UNITY_END();
}
