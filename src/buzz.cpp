#include "buzz.h"
#include "constants.h"
#include "foc_math.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace rotev {

// Fixed clkdiv chosen so TOP fits in the 16-bit PWM wrap register across the
// whole 1-4kHz buzzer range: at 200MHz sysclk / 8 = 25MHz PWM clock,
// TOP = 25MHz/1000Hz - 1 = 24999 (1kHz, largest TOP) down to 6249 (4kHz).
static constexpr float BUZZ_CLKDIV = 8.0f;

void buzzInit() {
  gpio_set_function(PIN_BUZZ, GPIO_FUNC_PWM);
  unsigned slice = pwm_gpio_to_slice_num(PIN_BUZZ);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_clkdiv(&c, BUZZ_CLKDIV);
  pwm_config_set_wrap(&c, 0xFFFF);  // provisional; buzzOn sets the real wrap per frequency
  pwm_init(slice, &c, false);        // don't start; buzzOff() leaves it disabled
  pwm_set_gpio_level(PIN_BUZZ, 0);
}

void buzzOn(uint16_t freq_hz) {
  uint16_t f = buzzClampFreq(freq_hz);
  unsigned slice = pwm_gpio_to_slice_num(PIN_BUZZ);
  uint32_t pwm_clk = (uint32_t)(clock_get_hz(clk_sys) / BUZZ_CLKDIV);
  uint16_t top = (uint16_t)(pwm_clk / f - 1);
  pwm_set_wrap(slice, top);
  pwm_set_gpio_level(PIN_BUZZ, top / 2);  // 50% duty
  // Known Pitfall (docs/PRD.md): pwm_hw->en is a single register shared by every
  // slice -- never blind-overwrite it. OR in only this slice's bit.
  pwm_hw->en |= (1u << slice);
}

void buzzOff() {
  unsigned slice = pwm_gpio_to_slice_num(PIN_BUZZ);
  pwm_hw->en &= ~(1u << slice);
  // Disabling the slice freezes the output at whatever level it held, which
  // at 50% duty is high about half the time. That leaves the piezo DC-biased
  // (harmless -- it is a capacitor, so only leakage flows -- but untidy), so
  // park the pin low: with level 0 the counter never compares true, and
  // zeroing the stopped counter forces the output to re-evaluate to low.
  pwm_set_gpio_level(PIN_BUZZ, 0);
  pwm_set_counter(slice, 0);
}

} // namespace rotev
