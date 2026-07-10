#include "button.h"
#include "hardware/gpio.h"

namespace rotev {

bool buttonRead(Button b) {
  return gpio_get(b == BTN_STOP ? PIN_BTN_STOP : PIN_BTN_GO);
}

} // namespace rotev
