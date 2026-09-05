#include "adc_ext.h"
#include "foc_math.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/platform.h"
#include "pico/time.h"

namespace rotev {

static constexpr uint8_t ADS1015_ADDR = 0x48;   // assumes ADDR pin tied to GND
static constexpr uint8_t REG_CONFIG = 0x01;
static constexpr uint8_t REG_CONVERSION = 0x00;
// 3300 SPS -> ~303us conversion; wait 4 ticks @ 100us/tick = 400us margin.
static constexpr uint8_t CONV_WAIT_TICKS = 4;

static spin_lock_t* s_lock;
static float s_vbus;
static float s_user[3];

static repeating_timer_t s_timer;
static uint8_t s_seq_idx = 0;
static uint8_t s_wait_ticks = 0;
static bool s_waiting = false;  // false = need to start a conversion; true = waiting on it
// TIMESTAMP of the last completed conversion, not a tick count. A tick counter
// incremented inside this callback cannot detect the failure it exists for: if
// the callback hangs (i2c_*_blocking has no timeout, so a slave holding SDA low
// blocks forever) the counter stops advancing too, and staleness reads false
// while s_vbus is frozen at its last good value. time_us_32() advances whether
// or not we run. Unsigned subtraction handles the 71-minute wrap.
static volatile uint32_t s_last_good_us = 0;

static bool adcExtTimerCB(repeating_timer_t*) {
  uint8_t ch = adcSeqChannel(s_seq_idx);

  if (!s_waiting) {
    uint16_t cfg = adcConfigForChannel(ch);
    uint8_t buf[3] = { REG_CONFIG, (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    if (i2c_write_blocking(i2c1, ADS1015_ADDR, buf, 3, false) < 0) {
      return true;  // retry next tick; do NOT advance into a read
    }
    s_wait_ticks = 0;
    s_waiting = true;
    return true;
  }

  if (++s_wait_ticks < CONV_WAIT_TICKS) return true;

  uint8_t ptr = REG_CONVERSION;
  uint8_t raw[2] = {0, 0};   // never publish uninitialised stack on a NAK
  bool ok = i2c_write_blocking(i2c1, ADS1015_ADDR, &ptr, 1, true) >= 0 &&
            i2c_read_blocking(i2c1, ADS1015_ADDR, raw, 2, false) >= 0;
  if (!ok) {
    s_waiting = false;
    return true;
  }
  s_last_good_us = time_us_32();
  int16_t reg = (int16_t)((raw[0] << 8) | raw[1]);
  float volts = adsRawToVolts(reg);

  uint32_t irq = spin_lock_blocking(s_lock);
  if (ch == 0) s_vbus = dividerToVbus(volts);
  else s_user[ch - 1] = volts;
  spin_unlock(s_lock, irq);

  s_seq_idx = (uint8_t)((s_seq_idx + 1) % 6);
  s_waiting = false;
  return true;
}

void adcExtInit() {
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(PIN_I2C1_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C1_SCL, GPIO_FUNC_I2C);
  // No gpio_pull_up() calls: this is a dedicated internal bus (not the user-facing
  // I2C0), assumed to have fixed pull-ups near the ADS1015 on the PCB.

  s_lock = spin_lock_init(spin_lock_claim_unused(true));
  s_vbus = VBUS_V;  // fallback nominal until the first real sample lands
  s_last_good_us = time_us_32();
  s_user[0] = s_user[1] = s_user[2] = 0.0f;

  add_repeating_timer_us(100, adcExtTimerCB, nullptr, &s_timer);
}

// vbus refreshes about every 1 ms (6-slot round robin, 3 of them channel 0).
// 20 ms is ~20 refreshes: far outside normal jitter, well inside the 250 ms
// fault window in foc.cpp.
bool __not_in_flash_func(adcExtVbusStale)() {
  return (uint32_t)(time_us_32() - s_last_good_us) > 20000u;
}

float __not_in_flash_func(adcExtVbus)() {
  uint32_t irq = spin_lock_blocking(s_lock);
  float v = s_vbus;
  spin_unlock(s_lock, irq);
  return v;
}

float adcExtUser(AdcChannel ch) {
  uint32_t irq = spin_lock_blocking(s_lock);
  float v = s_user[(int)ch];
  spin_unlock(s_lock, irq);
  return v;
}

} // namespace rotev
