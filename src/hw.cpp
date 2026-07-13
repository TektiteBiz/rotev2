#include "hw.h"
#include <initializer_list>
#include <Arduino.h>
#include "hardware/clocks.h"
#include "pico/stdlib.h"

namespace rotev {

void hwInit() {
  set_sys_clock_khz(150000, true);   // 150 MHz stock; overclock target tuned in bringup

  // nSLEEP pins as outputs, low (driver pairs asleep until motorEnable()).
  // PH pins are configured as PWM outputs by pwmInit() in focStart(), not
  // here -- EN is hardwired HIGH on the PCB (locked-antiphase drive, see
  // Drivers section in PRD.md), no GPIO control needed for it at all.
  for (uint32_t p : {PIN_NSLEEP_1, PIN_NSLEEP_2}) {
    gpio_init(p); gpio_set_dir(p, GPIO_OUT); gpio_put(p, 0);
  }

  // Buttons: input, pull-down (active-high).
  for (uint32_t p : {PIN_BTN_STOP, PIN_BTN_GO}) {
    gpio_init(p); gpio_set_dir(p, GPIO_IN); gpio_pull_down(p);
  }
}

void hwSetNsleep(Motor m, bool on) {
  gpio_put(m == MOTOR_1 ? PIN_NSLEEP_1 : PIN_NSLEEP_2, on ? 1 : 0);
}

} // namespace rotev
