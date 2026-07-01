#include "hw.h"
#include <initializer_list>
#include <Arduino.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"

namespace rotev {

void hwInit() {
  set_sys_clock_khz(150000, true);   // 150 MHz stock; overclock target tuned in bringup

  // Driver control pins (PH/EN) as outputs, low.
  const uint32_t outs[] = {PIN_ENA_1,PIN_PHA_1,PIN_ENB_1,PIN_PHB_1,
                           PIN_ENA_2,PIN_PHA_2,PIN_ENB_2,PIN_PHB_2,
                           PIN_NSLEEP_1,PIN_NSLEEP_2};
  for (uint32_t p : outs) { gpio_init(p); gpio_set_dir(p, GPIO_OUT); gpio_put(p, 0); }

  // Buttons: input, pull-down (active-high).
  for (uint32_t p : {PIN_BTN_STOP, PIN_BTN_GO}) {
    gpio_init(p); gpio_set_dir(p, GPIO_IN); gpio_pull_down(p);
  }
}

void hwSetNsleep(Motor m, bool on) {
  gpio_put(m == MOTOR_1 ? PIN_NSLEEP_1 : PIN_NSLEEP_2, on ? 1 : 0);
}

} // namespace rotev
