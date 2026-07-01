#include "led.h"
#include "constants.h"
#include "foc_math.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace rotev {

static const uint16_t LED_TOP = 3124; // ~ resolution; freq set by clkdiv

static void cfg(uint32_t pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(pin);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_wrap(&c, LED_TOP);
  // ~1 kHz: clkdiv = f_sys / ((LED_TOP+1) * 1000)
  pwm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / ((LED_TOP + 1) * 1000.0f));
  pwm_init(slice, &c, true);
  pwm_set_gpio_level(pin, LED_TOP); // active-low: start off
}

void ledInit() { cfg(PIN_LED_R); cfg(PIN_LED_G); cfg(PIN_LED_B); }

void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  pwm_set_gpio_level(PIN_LED_R, ledDuty(r, LED_TOP));
  pwm_set_gpio_level(PIN_LED_G, ledDuty(g, LED_TOP));
  pwm_set_gpio_level(PIN_LED_B, ledDuty(b, LED_TOP));
}

} // namespace rotev
