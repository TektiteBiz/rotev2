#include <unity.h>
#include <cmath>
#include "foc_math.h"
#include "constants.h"
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

void test_electrical_angle_scales_by_pole_pairs() {
  // theta_mech * POLE_PAIRS = 50 rad, wrapped to (-PI, PI]: 50 - 8*2*PI.
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 50.0f - 8.0f * 2.0f * (float)M_PI, electricalAngle(1.0f));
}
void test_electrical_angle_stays_bounded_for_large_input() {
  // Large accumulated mechanical angle (e.g. late in a multi-rotation
  // profile) must not be fed unbounded into sinf/cosf -- see foc_math.cpp
  // wrapAngle().
  float e = electricalAngle(1000.0f);
  TEST_ASSERT_TRUE(e > -(float)M_PI && e <= (float)M_PI);
}
void test_omega_estimator_constant_velocity() {
  OmegaEst s; omegaReset(s);
  float dt = 1.0f/12000.0f, w = 0.0f;
  float theta = 0.0f;
  for (int i = 0; i < 500; ++i) { theta += 100.0f*dt; w = omegaStep(s, theta, dt, 0.05f); }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, w); // converges toward 100 rad/s
}
void test_counts_to_amps_midscale_is_zero() {
  // 1.65V -> counts = 1.65/3.3*4095 = 2047.5
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, countsToAmps(2048));
}
void test_counts_to_amps_one_amp() {
  // V = 1.65 + 1.5 = 3.15 -> counts = 3.15/3.3*4095 = 3908.8
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, countsToAmps(3909));
}
void test_clamp_current_limits() {
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.1f, clampCurrent(5.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5, -1.1f, clampCurrent(-5.0f));
}
void test_led_duty_active_low() {
  TEST_ASSERT_EQUAL_UINT16(0, ledDuty(255, 3124));     // full on -> pin low
  TEST_ASSERT_EQUAL_UINT16(3124, ledDuty(0, 3124));    // off -> pin high
}
void test_omega_first_call_returns_zero() {
  OmegaEst s; omegaReset(s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, omegaStep(s, 1.0f, 0.001f, 0.1f));
}
void test_led_duty_midpoint() {
  // 128*3124/255 = 1568 (truncated); active-low duty = top - 1568
  TEST_ASSERT_EQUAL_UINT16((uint16_t)(3124 - 1568), ledDuty(128, 3124));
}

void test_buzz_clamp_within_range() {
  TEST_ASSERT_EQUAL_UINT16(2000, buzzClampFreq(2000));
}
void test_buzz_clamp_below_min() {
  TEST_ASSERT_EQUAL_UINT16(1000, buzzClampFreq(500));
}
void test_buzz_clamp_above_max() {
  TEST_ASSERT_EQUAL_UINT16(4000, buzzClampFreq(5000));
}
void test_ads_raw_to_volts_zero() {
  // raw16 = 0 -> 0V
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, adsRawToVolts(0));
}
void test_ads_raw_to_volts_positive_full_scale() {
  // 12-bit code 2047 (max positive), left-justified into upper 12 bits of 16-bit reg: 2047 << 4
  int16_t raw = (int16_t)(2047 << 4);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2047.0f * (4.096f / 2048.0f), adsRawToVolts(raw));
}
void test_ads_raw_to_volts_negative() {
  // 12-bit code -1 (0xFFF), left-justified: 0xFFF0 as int16_t
  int16_t raw = (int16_t)0xFFF0;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f * (4.096f / 2048.0f), adsRawToVolts(raw));
}
void test_divider_to_vbus_matches_ratio() {
  // 12V bus -> divider output = 12 * 2.2/(7.3+2.2) = 2.7789...V -> undo should recover 12V
  float v_div = 12.0f * (2200.0f / (7300.0f + 2200.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, dividerToVbus(v_div));
}
void test_adc_seq_channel_weights_ain0() {
  // Sequence: AIN0, AIN1, AIN0, AIN2, AIN0, AIN3 (repeating, length 6)
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(0));
  TEST_ASSERT_EQUAL_UINT8(1, adcSeqChannel(1));
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(2));
  TEST_ASSERT_EQUAL_UINT8(2, adcSeqChannel(3));
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(4));
  TEST_ASSERT_EQUAL_UINT8(3, adcSeqChannel(5));
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(6));  // wraps
}
void test_adc_config_for_channel_selects_mux() {
  // Channel 0 (AIN0) config: OS=1, MUX=100, PGA=001, MODE=1, DR=111, COMP_QUE=11 -> 0xC3E3
  TEST_ASSERT_EQUAL_UINT16(0xC3E3, adcConfigForChannel(0));
  TEST_ASSERT_EQUAL_UINT16(0xD3E3, adcConfigForChannel(1));  // AIN1: MUX=101
  TEST_ASSERT_EQUAL_UINT16(0xE3E3, adcConfigForChannel(2));  // AIN2: MUX=110
  TEST_ASSERT_EQUAL_UINT16(0xF3E3, adcConfigForChannel(3));  // AIN3: MUX=111
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
  RUN_TEST(test_electrical_angle_scales_by_pole_pairs);
  RUN_TEST(test_electrical_angle_stays_bounded_for_large_input);
  RUN_TEST(test_omega_estimator_constant_velocity);
  RUN_TEST(test_counts_to_amps_midscale_is_zero);
  RUN_TEST(test_counts_to_amps_one_amp);
  RUN_TEST(test_clamp_current_limits);
  RUN_TEST(test_led_duty_active_low);
  RUN_TEST(test_omega_first_call_returns_zero);
  RUN_TEST(test_led_duty_midpoint);
  RUN_TEST(test_buzz_clamp_within_range);
  RUN_TEST(test_buzz_clamp_below_min);
  RUN_TEST(test_buzz_clamp_above_max);
  RUN_TEST(test_ads_raw_to_volts_zero);
  RUN_TEST(test_ads_raw_to_volts_positive_full_scale);
  RUN_TEST(test_ads_raw_to_volts_negative);
  RUN_TEST(test_divider_to_vbus_matches_ratio);
  RUN_TEST(test_adc_seq_channel_weights_ain0);
  RUN_TEST(test_adc_config_for_channel_selects_mux);
  return UNITY_END();
}
