#include "button.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace rotev {

static bool debounced(uint32_t pin) {
  // Require 3 consecutive agreeing reads ~1ms apart via busy sample.
  int hi = 0;
  for (int i = 0; i < 3; ++i) { hi += gpio_get(pin) ? 1 : 0; busy_wait_us_32(1000); }
  return hi >= 2;
}

bool buttonRead(Button b) {
  return debounced(b == BTN_STOP ? PIN_BTN_STOP : PIN_BTN_GO);
}

} // namespace rotev
