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

// Parks BUZZ at a hard 0V.
//
// Clearing a slice's enable bit freezes its output at whatever level it held
// when the counter clock stopped, and neither a CC nor a CTR write brings it
// back down: the output flop only re-evaluates on a counter clock edge, and a
// disabled slice gets none. At 50% duty that leaves the pin stuck high about
// half the time, putting a steady 3.3V DC across the piezo. The piezo is a
// capacitor so almost no current flows, but the bias is not harmless: the
// diaphragm sits deflected with 3V3 rail ripple coupled straight across it
// (audible hiss/static), and it dumps its stored charge as a chirp when the
// board is powered down. Take the pin away from the PWM block entirely and
// drive it low from SIO instead -- that is the only way to guarantee 0V.
static void buzzParkLow() {
  gpio_put(PIN_BUZZ, 0);
  gpio_set_dir(PIN_BUZZ, GPIO_OUT);
  gpio_set_function(PIN_BUZZ, GPIO_FUNC_SIO);
}

void buzzInit() {
  unsigned slice = pwm_gpio_to_slice_num(PIN_BUZZ);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_clkdiv(&c, BUZZ_CLKDIV);
  pwm_config_set_wrap(&c, 0xFFFF);  // provisional; buzzOn sets the real wrap per frequency
  pwm_init(slice, &c, false);        // don't start; buzzOn() routes the pin over and enables
  pwm_set_gpio_level(PIN_BUZZ, 0);
  buzzParkLow();
}

void buzzOn(uint16_t freq_hz) {
  uint16_t f = buzzClampFreq(freq_hz);
  unsigned slice = pwm_gpio_to_slice_num(PIN_BUZZ);
  uint32_t pwm_clk = (uint32_t)(clock_get_hz(clk_sys) / BUZZ_CLKDIV);
  uint16_t top = (uint16_t)(pwm_clk / f - 1);
  pwm_set_wrap(slice, top);
  pwm_set_gpio_level(PIN_BUZZ, top / 2);  // 50% duty
  // Restart from 0 rather than wherever the last note left the counter: on a
  // retune to a higher frequency the stopped counter can already sit above
  // the new TOP, and the hardware would then run all the way to 0xFFFF before
  // wrapping -- a silent gap of up to ~2.6ms before the new note starts.
  pwm_set_counter(slice, 0);
  // Known Pitfall (docs/PRD.md): pwm_hw->en is a single register shared by every
  // slice -- never blind-overwrite it. OR in only this slice's bit.
  pwm_hw->en |= (1u << slice);
  gpio_set_function(PIN_BUZZ, GPIO_FUNC_PWM);  // hand the pin over only once it is toggling
}

void buzzOff() {
  // Take the pin back to SIO first: killing the slice while the pin is still
  // routed to PWM is exactly what freezes a high level onto the piezo.
  buzzParkLow();
  pwm_hw->en &= ~(1u << pwm_gpio_to_slice_num(PIN_BUZZ));
}

} // namespace rotev
