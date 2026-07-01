#pragma once
#include "constants.h"
#include "hardware/gpio.h"
#ifdef ENABLE_LOOP_TIMING
#include "hardware/structs/sio.h"
#endif
namespace rotev {
inline void debugTimingInit() {
#ifdef ENABLE_LOOP_TIMING
  gpio_init(PIN_LOOP_TIMING); gpio_set_dir(PIN_LOOP_TIMING, GPIO_OUT); gpio_put(PIN_LOOP_TIMING, 0);
#endif
}
inline void debugTimingHigh() {
#ifdef ENABLE_LOOP_TIMING
  sio_hw->gpio_set = 1u << PIN_LOOP_TIMING;
#endif
}
inline void debugTimingLow() {
#ifdef ENABLE_LOOP_TIMING
  sio_hw->gpio_clr = 1u << PIN_LOOP_TIMING;
#endif
}
} // namespace rotev
