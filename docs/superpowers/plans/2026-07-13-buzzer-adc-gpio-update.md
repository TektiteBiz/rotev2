# Buzzer, ADS1015 ADC, GPIO Renumbering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Incorporate the PRD's buzzer, ADS1015 external ADC, and GPIO-renumbering changes into the
RotEv2 library: new pin map, a buzzer module, a background-timer-driven ADS1015 driver feeding live
bus voltage into the FOC loop and exposing 3 user channels, and updated bringup/docs.

**Architecture:** Two new self-contained modules (`buzz.*`, `adc_ext.*`) follow the existing
one-file-one-responsibility pattern (`led.*`, `button.*`). Pure conversion/clamping math is factored
into `foc_math.*` (host-testable, matching the existing `countsToAmps`/`ledDuty` precedent). Hardware
I/O (I2C, PWM registers, the background timer) lives in the new `.cpp` files and is verified by
on-target compilation, matching how `pwm.cpp`/`adc.cpp` are handled today (no host harness for
hardware-touching code).

**Tech Stack:** PlatformIO, Arduino core (earlephilhower) with Pico SDK headers (`hardware/pwm.h`,
`hardware/i2c.h`, `pico/time.h`), Unity test framework for the native/host test env.

## Global Constraints

- GPIO pins (from PRD, updated 2026-07-13): `BTN_STOP=20, BTN_GO=21, nSLEEP_1=22, nSLEEP_2=23,
  I2C1_SDA=18, I2C1_SCL=19, BUZZ=4`.
- Buzzer: passive piezo, range 1000-4000 Hz, 50% duty cycle, GPIO4.
- ADS1015 ADC on I2C1 (GPIO18/19): AIN0 = bus voltage via divider (7.3kΩ high side, 2.2kΩ low
  side), sampled at ~1kHz for FOC inverse-park. AIN1-3 exposed to the user.
- ADC scheduling: automatic background timer on core0 (per approved design spec
  `docs/superpowers/specs/2026-07-13-buzzer-adc-gpio-update-design.md`), not user-pumped. I2C1 is
  not a user-facing bus (that's I2C0 on GPIO16/17), so no contention risk.
- Round-robin weighting: AIN0 gets 2 of every 3 slots; AIN1/AIN2/AIN3 share the remaining slot in
  rotation. Sequence: `AIN0, AIN1, AIN0, AIN2, AIN0, AIN3` (repeating, length 6).
- ADS1015: single-shot mode, ±4.096V PGA FSR, 3300 SPS data rate, I2C address `0x48` (assumed
  ADDR pin tied to GND — flagged as an open assumption to confirm on hardware).
- Vbus fallback: seed with the existing `VBUS_V` nominal constant until the first real ADS1015
  sample lands; no special-cased startup branch elsewhere.
- Public API additions: `buzzerOn(uint16_t freq_hz)`, `buzzerOff()`, `float adcRead(AdcChannel ch)`,
  `float busVoltage()`.
- Phase 1 bringup gains: buzzer tune played on GO press, `busVoltage()` added to serial telemetry.
- Known Pitfall (must respect): never blind-overwrite `pwm_hw->en` — always OR-in or AND-out a
  single slice's bit (see `docs/PRD.md` "Known Pitfalls" and existing `src/pwm.cpp`).

---

### Task 1: GPIO/constants update + README pin table

**Files:**
- Modify: `src/constants.h:62-66`
- Modify: `README.md:132-159` (GPIO / Bus Map table)

**Interfaces:**
- Produces: `PIN_BTN_STOP`, `PIN_BTN_GO`, `PIN_NSLEEP_1`, `PIN_NSLEEP_2` (renumbered),
  `PIN_I2C1_SDA`, `PIN_I2C1_SCL`, `PIN_BUZZ` (new), `enum AdcChannel : uint8_t { ADC_AIN1=0,
  ADC_AIN2=1, ADC_AIN3=2 }` (new), `VBUS_DIV_HIGH_OHMS`, `VBUS_DIV_LOW_OHMS`, `ADS1015_FSR_V`,
  `BUZZ_MIN_HZ`, `BUZZ_MAX_HZ` (new constants used by later tasks).

- [ ] **Step 1: Update pin constants and add new constants in `src/constants.h`**

Replace lines 62-66 (the current `PIN_BTN_STOP`/`PIN_NSLEEP`/`PIN_SOx` block) with:

```cpp
constexpr uint32_t PIN_BTN_STOP = 20, PIN_BTN_GO = 21;
constexpr uint32_t PIN_NSLEEP_1 = 22, PIN_NSLEEP_2 = 23;
constexpr uint32_t PIN_SOB_1 = 26, PIN_SOA_1 = 27, PIN_SOB_2 = 28,
                   PIN_SOA_2 = 29;
constexpr uint32_t ADC_SOB_1 = 0, ADC_SOA_1 = 1, ADC_SOB_2 = 2, ADC_SOA_2 = 3;
constexpr uint32_t PIN_LOOP_TIMING = 10;  // SPI1 SCK; bringup only

// --- Buzzer (passive piezo, GPIO4) ---
constexpr uint32_t PIN_BUZZ = 4;
constexpr uint16_t BUZZ_MIN_HZ = 1000, BUZZ_MAX_HZ = 4000;

// --- ADS1015 external ADC (I2C1, GPIO18/19) ---
constexpr uint32_t PIN_I2C1_SDA = 18, PIN_I2C1_SCL = 19;
constexpr float ADS1015_FSR_V = 4.096f;          // PGA full-scale range used for all channels
constexpr float VBUS_DIV_HIGH_OHMS = 7300.0f;    // bus-voltage divider high side
constexpr float VBUS_DIV_LOW_OHMS = 2200.0f;     // bus-voltage divider low side

enum AdcChannel : uint8_t { ADC_AIN1 = 0, ADC_AIN2 = 1, ADC_AIN3 = 2 };
```

Note the pin numbers changed (`PIN_BTN_STOP` was 19, now 20; `PIN_BTN_GO` was 20, now 21;
`PIN_NSLEEP_1` was 21, now 22; `PIN_NSLEEP_2` was 22, now 23) — this matches the PRD's updated GPIO
map. `PIN_SOB_1`/`PIN_SOA_1`/`PIN_SOB_2`/`PIN_SOA_2`/`ADC_SOx`/`PIN_LOOP_TIMING` are unchanged, just
carried over verbatim.

- [ ] **Step 2: Compile-check the native test env (constants-only change, should be unaffected)**

Run: `pio test -e native`
Expected: all existing tests still PASS (this change doesn't touch any function under test, just
constants — confirms nothing else in `foc_math.cpp`/`constants.h` relies on the old pin numbers).

- [ ] **Step 3: Update the GPIO / Bus Map table in `README.md`**

Replace the table rows for BTN_STOP/BTN_GO/nSLEEP_1/nSLEEP_2 (lines 150-153) and the `4-7` row
(line 140), and insert a new I2C1 row, so the full table (lines 134-158) reads:

```markdown
| Signal | GPIO | Notes |
|---|---|---|
| PHA_2 | 0 | Motor 2 phase A PWM (DRV8874, locked-antiphase; EN hardwired HIGH) |
| PHB_2 | 1 | Motor 2 phase B PWM |
| PHA_1 | 2 | Motor 1 phase A PWM |
| PHB_1 | 3 | Motor 1 phase B PWM |
| BUZZ | 4 | Passive piezo buzzer, 1-4kHz, 50% duty |
| — | 5-7 | Free GPIO (6, 7 also usable as general-purpose PWM-capable user GPIO) |
| LED_R | 8 | Red, active-low PWM |
| LED_G | 9 | Green, active-low PWM |
| SPI1_SCK / LOOP_TIMING | 10 | SPI1 SCK; also bringup loop-timing pin |
| SPI1_MOSI | 11 | SPI1 MOSI (user bus) |
| SPI1_MISO | 12 | SPI1 MISO (user bus) |
| SPI1_CS | 13 | SPI1 CS (user bus) |
| LED_B | 14 | Blue, active-low PWM |
| I2C0_SDA | 16 | Board has pull-ups fitted (user bus) |
| I2C0_SCL | 17 | Board has pull-ups fitted (user bus) |
| I2C1_SDA | 18 | ADS1015 ADC (internal, not user-facing) |
| I2C1_SCL | 19 | ADS1015 ADC (internal, not user-facing) |
| BTN_STOP | 20 | Active-high, internal pull-down |
| BTN_GO | 21 | Active-high, internal pull-down |
| nSLEEP_1 | 22 | Motor 1 driver sleep (active-low) |
| nSLEEP_2 | 23 | Motor 2 driver sleep (active-low) |
| SOB_1 / ADC0 | 26 | Motor 1 phase B current sense |
| SOA_1 / ADC1 | 27 | Motor 1 phase A current sense |
| SOB_2 / ADC2 | 28 | Motor 2 phase B current sense |
| SOA_2 / ADC3 | 29 | Motor 2 phase A current sense |

SPI1 (GPIO 10–13) and I2C0 (GPIO 16–17) are routed to headers and are usable as plain GPIO when not
needed as buses. GPIO 24/25 are general GPIO but cannot do PWM (they overlap the LED PWM slice).
```

- [ ] **Step 4: Commit**

```bash
git add src/constants.h README.md
git commit -m "Update GPIO pin map for buzzer/ADC, add new constants"
```

---

### Task 2: Pure math helpers (buzzer clamp, ADS1015 conversion, round-robin sequencing)

**Files:**
- Modify: `src/foc_math.h`
- Modify: `src/foc_math.cpp`
- Modify: `test/test_foc_math/test_main.cpp`

**Interfaces:**
- Consumes: `BUZZ_MIN_HZ`, `BUZZ_MAX_HZ`, `ADS1015_FSR_V`, `VBUS_DIV_HIGH_OHMS`,
  `VBUS_DIV_LOW_OHMS` from `constants.h` (Task 1).
- Produces: `uint16_t buzzClampFreq(uint16_t freq_hz)`, `float adsRawToVolts(int16_t raw16)`,
  `float dividerToVbus(float v_div)`, `uint8_t adcSeqChannel(uint8_t seq_idx)`,
  `uint16_t adcConfigForChannel(uint8_t ch)` — all consumed by Task 3/4's hardware modules.

- [ ] **Step 1: Write the failing tests**

Add to `test/test_foc_math/test_main.cpp` (insert new test functions before `int main()`, after the
existing `test_led_duty_midpoint`):

```cpp
void test_buzz_clamp_within_range() {
  TEST_ASSERT_EQUAL_UINT16(2000, buzzClampFreq(2000));
}
void test_buzz_clamp_below_min() {
  TEST_ASSERT_EQUAL_UINT16(1000, buzzClampFreq(500));
}
void test_buzz_clamp_above_max() {
  TEST_ASSERT_EQUAL_UINT16(4000, buzzClampFreq(5000));
}
void test_ads_raw_to_volts_zero() {
  // raw16 = 0 -> 0V
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, adsRawToVolts(0));
}
void test_ads_raw_to_volts_positive_full_scale() {
  // 12-bit code 2047 (max positive), left-justified into upper 12 bits of 16-bit reg: 2047 << 4
  int16_t raw = (int16_t)(2047 << 4);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2047.0f * (4.096f / 2048.0f), adsRawToVolts(raw));
}
void test_ads_raw_to_volts_negative() {
  // 12-bit code -1 (0xFFF), left-justified: 0xFFF0 as int16_t
  int16_t raw = (int16_t)0xFFF0;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f * (4.096f / 2048.0f), adsRawToVolts(raw));
}
void test_divider_to_vbus_matches_ratio() {
  // 12V bus -> divider output = 12 * 2.2/(7.3+2.2) = 2.7789...V -> undo should recover 12V
  float v_div = 12.0f * (2200.0f / (7300.0f + 2200.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.0f, dividerToVbus(v_div));
}
void test_adc_seq_channel_weights_ain0() {
  // Sequence: AIN0, AIN1, AIN0, AIN2, AIN0, AIN3 (repeating, length 6)
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(0));
  TEST_ASSERT_EQUAL_UINT8(1, adcSeqChannel(1));
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(2));
  TEST_ASSERT_EQUAL_UINT8(2, adcSeqChannel(3));
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(4));
  TEST_ASSERT_EQUAL_UINT8(3, adcSeqChannel(5));
  TEST_ASSERT_EQUAL_UINT8(0, adcSeqChannel(6));  // wraps
}
void test_adc_config_for_channel_selects_mux() {
  // Channel 0 (AIN0) config: OS=1, MUX=100, PGA=001, MODE=1, DR=111, COMP_QUE=11 -> 0xC3E3
  TEST_ASSERT_EQUAL_UINT16(0xC3E3, adcConfigForChannel(0));
  TEST_ASSERT_EQUAL_UINT16(0xD3E3, adcConfigForChannel(1));  // AIN1: MUX=101
  TEST_ASSERT_EQUAL_UINT16(0xE3E3, adcConfigForChannel(2));  // AIN2: MUX=110
  TEST_ASSERT_EQUAL_UINT16(0xF3E3, adcConfigForChannel(3));  // AIN3: MUX=111
}
```

Add the corresponding `RUN_TEST(...)` lines inside `main()`, right before `return UNITY_END();`:

```cpp
  RUN_TEST(test_buzz_clamp_within_range);
  RUN_TEST(test_buzz_clamp_below_min);
  RUN_TEST(test_buzz_clamp_above_max);
  RUN_TEST(test_ads_raw_to_volts_zero);
  RUN_TEST(test_ads_raw_to_volts_positive_full_scale);
  RUN_TEST(test_ads_raw_to_volts_negative);
  RUN_TEST(test_divider_to_vbus_matches_ratio);
  RUN_TEST(test_adc_seq_channel_weights_ain0);
  RUN_TEST(test_adc_config_for_channel_selects_mux);
```

- [ ] **Step 2: Run tests to verify they fail to compile/link (functions don't exist yet)**

Run: `pio test -e native`
Expected: FAIL — compile error, `buzzClampFreq`/`adsRawToVolts`/`dividerToVbus`/`adcSeqChannel`/
`adcConfigForChannel` not declared.

- [ ] **Step 3: Add declarations to `src/foc_math.h`**

Insert before the closing `} // namespace rotev` (after the existing `ledDuty` declaration):

```cpp
// Buzzer frequency clamp (see constants.h BUZZ_MIN_HZ/BUZZ_MAX_HZ).
uint16_t buzzClampFreq(uint16_t freq_hz);

// ADS1015 raw 16-bit register value (12-bit result left-justified) -> volts at
// the ±4.096V PGA setting used for every channel.
float adsRawToVolts(int16_t raw16);

// Undoes the 7.3k/2.2k bus-voltage divider to recover the actual bus voltage.
float dividerToVbus(float v_div);

// Weighted round-robin channel sequence for the ADS1015 background sampler:
// AIN0 gets 2 of every 3 slots (bus voltage needs ~1kHz; user channels don't).
// Returns 0=AIN0, 1=AIN1, 2=AIN2, 3=AIN3 for sequence index seq_idx (wraps mod 6).
uint8_t adcSeqChannel(uint8_t seq_idx);

// ADS1015 single-shot config register value (16-bit) to start a conversion on
// the given channel (0=AIN0..3=AIN3): OS=1 (start), PGA=+-4.096V, MODE=single-shot,
// DR=3300SPS, COMP_QUE=11 (comparator disabled).
uint16_t adcConfigForChannel(uint8_t ch);
```

- [ ] **Step 4: Implement in `src/foc_math.cpp`**

Insert before the closing `} // namespace rotev` (after the existing `ledDuty` definition):

```cpp
uint16_t buzzClampFreq(uint16_t freq_hz) {
  if (freq_hz < BUZZ_MIN_HZ) return BUZZ_MIN_HZ;
  if (freq_hz > BUZZ_MAX_HZ) return BUZZ_MAX_HZ;
  return freq_hz;
}

float adsRawToVolts(int16_t raw16) {
  int16_t code12 = raw16 >> 4;  // arithmetic shift preserves sign
  return (float)code12 * (ADS1015_FSR_V / 2048.0f);
}

float dividerToVbus(float v_div) {
  return v_div * (VBUS_DIV_HIGH_OHMS + VBUS_DIV_LOW_OHMS) / VBUS_DIV_LOW_OHMS;
}

uint8_t adcSeqChannel(uint8_t seq_idx) {
  static constexpr uint8_t kSeq[6] = {0, 1, 0, 2, 0, 3};
  return kSeq[seq_idx % 6];
}

uint16_t adcConfigForChannel(uint8_t ch) {
  // OS=1, MUX=100+ch, PGA=001 (+-4.096V), MODE=1 (single-shot), DR=111 (3300SPS),
  // COMP_MODE=0, COMP_POL=0, COMP_LAT=0, COMP_QUE=11 (disabled)
  uint16_t mux = (uint16_t)(4 + ch);  // 100..111
  return (uint16_t)(0x8000 |            // OS
                     (mux << 12) |
                     (0x1 << 9)  |      // PGA = 001
                     (0x1 << 8)  |      // MODE = single-shot
                     (0x7 << 5)  |      // DR = 111 (3300SPS)
                     0x3);              // COMP_QUE = 11
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS — all tests including the 9 new ones green.

- [ ] **Step 6: Commit**

```bash
git add src/foc_math.h src/foc_math.cpp test/test_foc_math/test_main.cpp
git commit -m "Add buzzer clamp and ADS1015 conversion/sequencing math with tests"
```

---

### Task 3: Buzzer module + public API

**Files:**
- Create: `src/buzz.h`
- Create: `src/buzz.cpp`
- Modify: `src/rotev.h`
- Modify: `src/rotev.cpp`

**Interfaces:**
- Consumes: `PIN_BUZZ` (Task 1), `buzzClampFreq` (Task 2).
- Produces: `void buzzInit()`, `void buzzOn(uint16_t freq_hz)`, `void buzzOff()` (internal); public
  `rotev::buzzerOn(uint16_t freq_hz)`, `rotev::buzzerOff()`.

- [ ] **Step 1: Create `src/buzz.h`**

```cpp
#pragma once
#include <cstdint>
namespace rotev {
void buzzInit();
void buzzOn(uint16_t freq_hz);
void buzzOff();
}
```

- [ ] **Step 2: Create `src/buzz.cpp`**

```cpp
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
}

} // namespace rotev
```

- [ ] **Step 3: Add public API to `src/rotev.h`**

In `src/rotev.h`, add after the `setLagComp` declaration:

```cpp
void  buzzerOn(uint16_t freq_hz);  // clamped to [1000,4000] Hz, 50% duty
void  buzzerOff();
```

- [ ] **Step 4: Wire into `src/rotev.cpp`**

Add `#include "buzz.h"` to the includes at the top of `src/rotev.cpp`. In `begin()`, add
`buzzInit();` after `focStart();`:

```cpp
void begin() {
  hwInit();
  ledInit();
  focStart();
  buzzInit();
}
```

Add the two new public functions after `setLagComp`:

```cpp
void buzzerOn(uint16_t freq_hz) { buzzOn(freq_hz); }
void buzzerOff() { buzzOff(); }
```

- [ ] **Step 5: Compile-verify on target**

Run: `cd bringup && pio run -e phase1`
Expected: builds successfully (the library, including the new `buzz.cpp`, compiles as part of the
`symlink://../` lib dependency).

- [ ] **Step 6: Commit**

```bash
git add src/buzz.h src/buzz.cpp src/rotev.h src/rotev.cpp
git commit -m "Add buzzer module and public buzzerOn/buzzerOff API"
```

---

### Task 4: ADS1015 driver module + public API

**Files:**
- Create: `src/adc_ext.h`
- Create: `src/adc_ext.cpp`
- Modify: `src/rotev.h`
- Modify: `src/rotev.cpp`

**Interfaces:**
- Consumes: `PIN_I2C1_SDA`, `PIN_I2C1_SCL`, `VBUS_V` (fallback), `AdcChannel` (Task 1);
  `adsRawToVolts`, `dividerToVbus`, `adcSeqChannel`, `adcConfigForChannel` (Task 2).
- Produces: `void adcExtInit()`, `float adcExtVbus()`, `float adcExtUser(AdcChannel ch)`
  (internal); public `float rotev::adcRead(AdcChannel ch)`, `float rotev::busVoltage()`. Also
  consumed by Task 5 (`adcExtVbus()` used inside `foc.cpp`).

- [ ] **Step 1: Create `src/adc_ext.h`**

```cpp
#pragma once
#include "constants.h"
namespace rotev {
void  adcExtInit();          // configures I2C1 + starts the background sampling timer
float adcExtVbus();          // last cached bus voltage (volts), non-blocking
float adcExtUser(AdcChannel ch);  // last cached AIN1/2/3 sample (volts), non-blocking
}
```

- [ ] **Step 2: Create `src/adc_ext.cpp`**

```cpp
#include "adc_ext.h"
#include "foc_math.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
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

static bool adcExtTimerCB(repeating_timer_t*) {
  uint8_t ch = adcSeqChannel(s_seq_idx);

  if (!s_waiting) {
    uint16_t cfg = adcConfigForChannel(ch);
    uint8_t buf[3] = { REG_CONFIG, (uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF) };
    i2c_write_blocking(i2c1, ADS1015_ADDR, buf, 3, false);
    s_wait_ticks = 0;
    s_waiting = true;
    return true;
  }

  if (++s_wait_ticks < CONV_WAIT_TICKS) return true;

  uint8_t ptr = REG_CONVERSION;
  i2c_write_blocking(i2c1, ADS1015_ADDR, &ptr, 1, true);  // no stop -> repeated start
  uint8_t raw[2];
  i2c_read_blocking(i2c1, ADS1015_ADDR, raw, 2, false);
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
  s_user[0] = s_user[1] = s_user[2] = 0.0f;

  add_repeating_timer_us(100, adcExtTimerCB, nullptr, &s_timer);
}

float adcExtVbus() {
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
```

- [ ] **Step 3: Add public API to `src/rotev.h`**

Add after the `buzzerOff` declaration (from Task 3):

```cpp
float adcRead(AdcChannel ch);  // ADC_AIN1/2/3, last cached sample in volts, non-blocking
float busVoltage();            // last cached bus voltage in volts, non-blocking
```

- [ ] **Step 4: Wire into `src/rotev.cpp`**

Add `#include "adc_ext.h"` to the includes. In `begin()`, add `adcExtInit();` after `buzzInit();`:

```cpp
void begin() {
  hwInit();
  ledInit();
  focStart();
  buzzInit();
  adcExtInit();
}
```

Add the two new public functions after `buzzerOff`:

```cpp
float adcRead(AdcChannel ch) { return adcExtUser(ch); }
float busVoltage() { return adcExtVbus(); }
```

- [ ] **Step 5: Compile-verify on target**

Run: `cd bringup && pio run -e phase1`
Expected: builds successfully.

- [ ] **Step 6: Commit**

```bash
git add src/adc_ext.h src/adc_ext.cpp src/rotev.h src/rotev.cpp
git commit -m "Add ADS1015 driver with background sampling and public adcRead/busVoltage API"
```

---

### Task 5: Wire live bus voltage into the FOC loop

**Files:**
- Modify: `src/foc.cpp:1-108` (includes + `controlStep`)

**Interfaces:**
- Consumes: `adcExtVbus()` (Task 4).
- Produces: `controlStep` now uses a per-call live `vbus` instead of the `VBUS_V` constant; no
  change to `foc.h`'s public signatures.

- [ ] **Step 1: Add the include**

In `src/foc.cpp`, add `#include "adc_ext.h"` alongside the existing includes (after `#include
"adc.h"`).

- [ ] **Step 2: Read live Vbus at the top of `controlStep` and thread it through**

In `src/foc.cpp`, `controlStep` currently starts:

```cpp
static void __not_in_flash_func(controlStep)(Motor m, AB i) {
  Setpoint sp;
  bool lag;
  uint32_t irq = spin_lock_blocking(s_lock);
```

Change to:

```cpp
static void __not_in_flash_func(controlStep)(Motor m, AB i) {
  float vbus = adcExtVbus();
  Setpoint sp;
  bool lag;
  uint32_t irq = spin_lock_blocking(s_lock);
```

Then replace every remaining use of `VBUS_V` inside `controlStep` with `vbus`. Concretely, these
three lines:

```cpp
    uq = piStep(s_piq[m], sp.iq_cmd - dq.q, KP, KI, dt, VBUS_V);
    ud = piStep(s_pid[m], 0.0f      - dq.d, KP, KI, dt, VBUS_V);
```

become:

```cpp
    uq = piStep(s_piq[m], sp.iq_cmd - dq.q, KP, KI, dt, vbus);
    ud = piStep(s_pid[m], 0.0f      - dq.d, KP, KI, dt, vbus);
```

and:

```cpp
    float ud_mag = fabsf(ud);
    if (ud_mag > VBUS_V) {
      ud *= VBUS_V / ud_mag;
      uq = 0.0f;
    } else {
      float uq_budget = sqrtf(VBUS_V * VBUS_V - ud * ud);
```

becomes:

```cpp
    float ud_mag = fabsf(ud);
    if (ud_mag > vbus) {
      ud *= vbus / ud_mag;
      uq = 0.0f;
    } else {
      float uq_budget = sqrtf(vbus * vbus - ud * ud);
```

and finally:

```cpp
  AB v = inversePark(ud, uq, theta_e, VBUS_V);     // normalized duties [-1,1]
```

becomes:

```cpp
  AB v = inversePark(ud, uq, theta_e, vbus);       // normalized duties [-1,1]
```

Leave the `sp.openloop` branch's `uq = sp.iq_cmd;` line and everything in the `sp.ab_mode` branch
untouched — those don't reference `VBUS_V`. Also leave `focSetVoltageAB`/`motorWriteVoltageAB` (in
`rotev.cpp`) using the `VBUS_V` constant as-is — that's a separate bringup/testing helper (phase1c)
outside this change's scope; only the closed-loop `controlStep` path gets live Vbus.

- [ ] **Step 3: Compile-verify on target**

Run: `cd bringup && pio run -e phase1 -e phase2 -e phase3 -e phase4`
Expected: all four environments build successfully.

- [ ] **Step 4: Run the native test suite to confirm no regression**

Run: `pio test -e native`
Expected: PASS — `foc.cpp` isn't part of the native build filter, but this confirms nothing else
broke.

- [ ] **Step 5: Commit**

```bash
git add src/foc.cpp
git commit -m "Feed live ADS1015 bus voltage into the FOC control loop"
```

---

### Task 6: Phase 1 bringup additions (buzzer tune on GO, bus voltage telemetry)

**Files:**
- Modify: `bringup/phase1_hw.h`

**Interfaces:**
- Consumes: `rotev::buzzerOn`, `rotev::buzzerOff`, `rotev::busVoltage` (Tasks 3-4).

- [ ] **Step 1: Add the buzzer tune and bus voltage telemetry**

Replace the full contents of `bringup/phase1_hw.h` with:

```cpp
#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Basic hardware verification: enable/disable the drivers, log idle phase
// currents and bus voltage, confirm the LED/buttons/buzzer work. No PWM is
// applied to the motor windings beyond what the driver requires to
// acknowledge enable.

static void playGoTune() {
  const uint16_t notes[] = {1000, 1500, 2000};
  for (uint16_t f : notes) {
    buzzerOn(f);
    delay(80);
  }
  buzzerOff();
}

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);   // nSLEEP high; no motion commanded
  motorEnable(MOTOR_2);
}

void loop() {
  bool go   = buttonPressed(BTN_GO);
  bool stop = buttonPressed(BTN_STOP);

  Serial.print(">motorCurrentA(MOTOR_1):"); Serial.print(motorCurrentA(MOTOR_1));
  Serial.print(",motorCurrentB(MOTOR_1):"); Serial.print(motorCurrentB(MOTOR_1));
  Serial.print(",motorCurrentA(MOTOR_2):"); Serial.print(motorCurrentA(MOTOR_2));
  Serial.print(",motorCurrentB(MOTOR_2):"); Serial.print(motorCurrentB(MOTOR_2));
  Serial.print(",busVoltage:"); Serial.println(busVoltage());

  if (go) { ledColor(0, 255, 0); playGoTune(); }
  if (stop) ledColor(255, 0, 0);
  delay(50);
}
```

- [ ] **Step 2: Compile-verify on target**

Run: `cd bringup && pio run -e phase1`
Expected: builds successfully.

- [ ] **Step 3: Commit**

```bash
git add bringup/phase1_hw.h
git commit -m "Add buzzer tune and bus voltage telemetry to Phase 1 bringup"
```

---

### Task 7: README documentation (Buzzer, ADC/AIN, Bus Voltage sections) + full verification

**Files:**
- Modify: `README.md`

**Interfaces:**
- None (documentation only).

- [ ] **Step 1: Add a Buzzer section to the Public API part of `README.md`**

Insert after the "### LED" section (after line 104, before "### Buttons"):

```markdown
### Buzzer

```cpp
void buzzerOn(uint16_t freq_hz);
void buzzerOff();
```

Drives the passive piezo buzzer on GPIO4 at 50% duty cycle. `freq_hz` is clamped to 1000-4000 Hz.
Calling `buzzerOn()` again while already sounding retunes the frequency without needing to call
`buzzerOff()` first.
```

- [ ] **Step 2: Add an ADC/AIN section**

Insert a new section after "### Current Telemetry" (after line 122, before "### Enums"):

```markdown
### External ADC (Bus Voltage + User Channels)

```cpp
float adcRead(AdcChannel ch);  // ADC_AIN1 / ADC_AIN2 / ADC_AIN3
float busVoltage();
```

An ADS1015 I2C ADC (on the internal I2C1 bus, GPIO18/19 — not user-facing) samples the motor bus
voltage (via a 7.3kΩ/2.2kΩ divider) and 3 user-facing channels (AIN1-3) in the background, fully
automatically — there is nothing to call or poll. A repeating timer started by `begin()` round-robins
the 4 channels, weighted so bus voltage updates at roughly 1kHz (needed internally by the FOC loop's
inverse-Park) while each user channel updates at roughly 150-250Hz (ample for telemetry/display use).
`adcRead()` and `busVoltage()` return the most recently cached sample in volts; both are
non-blocking.
```

- [ ] **Step 3: Update the `Enums` section** to include `AdcChannel`

In the "### Enums" section, add the new enum after `Button`:

```cpp
enum AdcChannel : uint8_t { ADC_AIN1 = 0, ADC_AIN2 = 1, ADC_AIN3 = 2 };
```

- [ ] **Step 4: Update the "Bus voltage is assumed to be 12V" line in the FOC/PI Tuning section**

Replace:

```markdown
Bus voltage is assumed to be **12 V** (used behind the inverse-Park transform to normalize duty cycle). This will be replaced by a live ADC read in a future revision.
```

with:

```markdown
Bus voltage is read live from the ADS1015 (`busVoltage()`, ~1kHz) and used behind the inverse-Park
transform to normalize duty cycle. A nominal 12V fallback is used only until the first real ADC
sample lands at boot.
```

- [ ] **Step 5: Full verification build**

Run each of the following and confirm all succeed:

```bash
pio test -e native
cd bringup && pio run -e phase1 -e phase1b -e phase1c -e phase2 -e phase3 -e phase4
```

Expected: native tests all PASS; all six bringup environments build without error.

- [ ] **Step 6: Commit**

```bash
git add README.md
git commit -m "Document buzzer, external ADC, and live bus voltage in README"
```

---

## Self-Review Notes

- **Spec coverage:** GPIO renumbering (Task 1), buzzer module + API (Task 3), ADS1015 driver +
  round-robin + Vbus feed + user channels (Tasks 2, 4, 5), Phase 1 bringup additions (Task 6),
  README docs (Task 7) — all sections of the design spec are covered.
- **Vbus fallback:** confirmed handled without special-casing (Task 4, `s_vbus = VBUS_V` at init).
- **`motorWriteVoltageAB`/`focSetVoltageAB`** intentionally left on the `VBUS_V` constant (Task 5
  note) — out of scope; it's a bringup-only direct-duty helper, not part of the closed-loop path
  the PRD asked to fix.
- **Type consistency:** `AdcChannel` defined once in `constants.h` (Task 1), used identically as
  the parameter type for `adcExtUser`/`adcRead` (Tasks 4, 7) — no renaming drift.
