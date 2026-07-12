#include "led.h"
#include "constants.h"
#include "foc_math.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace rotev {

static const uint16_t LED_TOP = 3124; // ~ resolution; freq set by clkdiv

// pwm_init() resets the whole slice's counter-compare register (both
// channels) to 0 every time it's called. PIN_LED_R and PIN_LED_G share a
// slice on this GPIO layout, so each unique slice must be pwm_init'd exactly
// once; channel levels are only ever touched afterward via
// pwm_set_gpio_level (a masked read-modify-write), never by re-running
// pwm_init on an already-configured slice.
static void initSlice(uint slice) {
  pwm_config c = pwm_get_default_config();
  pwm_config_set_wrap(&c, LED_TOP);
  // ~1 kHz: clkdiv = f_sys / ((LED_TOP+1) * 1000)
  pwm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / ((LED_TOP + 1) * 1000.0f));
  pwm_init(slice, &c, true);
}

void ledInit() {
  gpio_set_function(PIN_LED_R, GPIO_FUNC_PWM);
  gpio_set_function(PIN_LED_G, GPIO_FUNC_PWM);
  gpio_set_function(PIN_LED_B, GPIO_FUNC_PWM);

  uint sliceR = pwm_gpio_to_slice_num(PIN_LED_R);
  uint sliceG = pwm_gpio_to_slice_num(PIN_LED_G);
  uint sliceB = pwm_gpio_to_slice_num(PIN_LED_B);
  initSlice(sliceR);
  if (sliceG != sliceR) initSlice(sliceG);
  if (sliceB != sliceR && sliceB != sliceG) initSlice(sliceB);

  ledSet(0, 0, 0); // active-low: start off
}

void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  pwm_set_gpio_level(PIN_LED_R, ledDuty(r, LED_TOP));
  pwm_set_gpio_level(PIN_LED_G, ledDuty(g, LED_TOP));
  pwm_set_gpio_level(PIN_LED_B, ledDuty(b, LED_TOP));
}

} // namespace rotev
