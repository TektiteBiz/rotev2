# RotEv2 FOC Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a published PlatformIO/Arduino library for the Tektite RotEv2 PCB (RP2354) implementing dual-core, closed-loop-current / open-loop-position FOC for two 2-phase steppers, plus RGB LED and button support, validated through a 4-phase hardware bringup.

**Architecture:** Public free-function API in `namespace rotev` on core0; a real-time FOC control loop on core1 driven by a 24 kHz center-aligned PWM-wrap IRQ (alternating motors → 12 kHz each). Pure control math is isolated in a hardware-free `foc_math` unit (host-unit-tested with Unity); hardware modules use Pico SDK `<hardware/*.h>` headers and are validated on-target via bringup sketches.

**Tech Stack:** C++17, earlephilhower arduino-pico core (RP2350/RP2354), Pico SDK (`hardware/pwm.h`, `hardware/adc.h`, `hardware/irq.h`, `hardware/clocks.h`, `pico/multicore.h`, `hardware/sync.h`), PlatformIO, Unity test framework (native env).

## Global Constraints

- **MCU / board:** RP2354; PlatformIO `board = generic_rp2350` (or Pico 2 variant), `board_build.core = earlephilhower`, `platform = raspberrypi` (maxgerhardt fork).
- **Language:** C++17. Public symbols live in `namespace rotev`. Enums: `Motor { MOTOR_1, MOTOR_2 }`, `Button { BTN_STOP, BTN_GO }`.
- **`foc_math` purity:** `src/foc_math.*` may include ONLY `<cmath>`, `<cstdint>`, `<cstddef>`. No Arduino/Pico headers — it must compile and unit-test under `platform = native`.
- **Concurrency contract:** the library owns core1 via `multicore_launch_core1`. User sketches MUST NOT define `setup1()`/`loop1()`. Documented in README.
- **Cross-core safety:** all core0↔core1 shared state passes through the spinlock-protected setpoint/telemetry structs in `foc.*`; no other shared mutable state.
- **Verbatim physical constants (motor 14HS11-1004):** pole pairs `POLE_PAIRS = 50` (1.8°/step); `PHASE_R = 3.5f` Ω; `PHASE_L = 0.0035f` H (Ld = Lq); rated 1.0 A.
- **Verbatim control constants:** `BANDWIDTH = 1000.0f` rad/s; `KP = BANDWIDTH * PHASE_L = 3.5f`; `KI = BANDWIDTH * PHASE_R = 3500.0f`.
- **Verbatim sensing constants:** `SHUNT_OHMS = 0.015f`; `INA_GAIN = 100.0f` (INA186A3); `ISENSE_REF_V = 1.65f`; `ADC_VREF = 3.3f`; `ADC_MAX = 4095.0f`; derived `VOLTS_PER_AMP = SHUNT_OHMS * INA_GAIN = 1.5f`; measurable range ≈ ±1.1 A → `IMAX_A = 1.1f` (current command clamp).
- **Verbatim PWM/loop constants:** `PWM_HZ = 24000`, per-motor loop 12 kHz (alternate); `ADC_OVERSAMPLE = 4` (default). PWM `TOP = clock_get_hz(clk_sys) / (2 * PWM_HZ)` (phase-correct), computed at runtime.
- **Verbatim GPIO map:** ENA_2=0, PHA_2=1, ENB_2=2, PHB_2=3, ENA_1=4, PHA_1=5, ENB_1=6, PHB_1=7, LED_R=8, LED_G=9, LED_B=14, BTN_STOP=19, BTN_GO=20, nSLEEP_1=21, nSLEEP_2=22, SOB_1=26/ADC0, SOA_1=27/ADC1, SOB_2=28/ADC2, SOA_2=29/ADC3. SPI1: SCK=10, MOSI=11, MISO=12, CS=13 (user; bringup debug). I2C0: SDA=16, SCL=17. `LOOP_TIMING_PIN = 10` (bringup only, behind `ENABLE_LOOP_TIMING`).
- **LED polarity:** active-low (PWM drives cathodes): brightness 255 = full on = pin held low; duty inverted internally.
- **Bus voltage:** `VBUS_V = 12.0f`, isolated behind `inversePark(...)` so a future ADC bus read is a one-line change.
- **Commit style:** commit after every green step. Never work on `main` — a feature branch + pre-feature tag is created before Task 1.

---

## File Structure

```
rotev2/
  platformio.ini              # native test env + rp2350 build env (for compile checks)
  library.json                # PlatformIO manifest
  library.properties          # Arduino manifest
  keywords.txt                # Arduino IDE highlighting
  src/
    rotev.h        rotev.cpp   # public rotev:: API (delegates to internals)
    constants.h                # all verbatim constants + Motor/Button enums (header-only)
    foc_math.h     foc_math.cpp# PURE math: park, inverse_park, PIState, omega estimator, adc scaling, led duty
    hw.h           hw.cpp      # clock/overclock, pin init, nSLEEP control
    pwm.h          pwm.cpp     # center-aligned 24kHz PWM setup + per-phase set(voltage, vbus)
    adc.h          adc.cpp     # oversampled 2-channel sampling + counts->amps
    foc.h          foc.cpp     # core1 launch, PWM-wrap ISR, per-motor control step, shared setpoint/telemetry
    led.h          led.cpp     # active-low RGB PWM
    button.h       button.cpp  # pull-down + debounced reads
    debug.h        debug.cpp   # LOOP_TIMING_PIN toggling
  examples/
    Basic/Basic.ino            # short published usage example
  bringup/
    platformio.ini             # self-contained project; lib_deps = symlink://../..
    src/main.cpp               # selects phase via build flag (-DPHASE=N)
    phase1_hw.h  phase2_openloop.h  phase3_foc.h  phase4_full.h
  test/
    test_foc_math/test_main.cpp# Unity native unit tests for foc_math
  docs/PRD.md  docs/BRINGUP.md  docs/superpowers/...
  README.md
```

---

### Task 0: Branch, tag, and project scaffolding

**Files:**
- Create: `platformio.ini`, `library.json`, `library.properties`, `keywords.txt`, `src/constants.h`
- Test: `test/test_foc_math/test_main.cpp` (skeleton)

**Interfaces:**
- Produces: `constants.h` with all Global-Constraints constants and `enum Motor`, `enum Button`; a buildable `native` test env.

- [ ] **Step 1: Safety branch + tag**

```bash
cd /c/Users/bvikramadity/Development/nv/rotev2
git tag v0-pre-foc-library
git checkout -b feature/foc-library
```

- [ ] **Step 2: Write `library.json`**

```json
{
  "name": "RotEv2",
  "version": "0.1.0",
  "description": "Dual-core open-loop-position / closed-loop-current stepper FOC for the Tektite RotEv2 (RP2354).",
  "keywords": "foc, stepper, rp2350, rp2354, motor, control",
  "repository": { "type": "git", "url": "https://github.com/Nv7-Github/rotev2.git" },
  "frameworks": "arduino",
  "platforms": "raspberrypi",
  "headers": "rotev.h",
  "export": { "exclude": ["bringup", "docs", "test"] }
}
```

- [ ] **Step 3: Write `library.properties`**

```
name=RotEv2
version=0.1.0
author=Tektite
maintainer=Nv7-Github
sentence=Dual-core stepper FOC for the Tektite RotEv2 (RP2354).
paragraph=Open-loop-position / closed-loop-current field-oriented control for two 2-phase steppers, plus RGB LED and buttons.
category=Device Control
url=https://github.com/Nv7-Github/rotev2
architectures=rp2040
includes=rotev.h
```

- [ ] **Step 4: Write `platformio.ini`**

```ini
[env:native]
platform = native
test_framework = unity
build_flags = -std=c++17 -I src
build_src_filter = +<foc_math.cpp>

[env:rp2350]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = generic_rp2350
board_build.core = earlephilhower
framework = arduino
build_flags = -std=gnu++17
```

- [ ] **Step 5: Write `src/constants.h`**

```cpp
#pragma once
#include <cstdint>

namespace rotev {

enum Motor : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };

// --- Motor 14HS11-1004 ---
constexpr int   POLE_PAIRS = 50;      // 1.8 deg/step -> 200 steps/rev
constexpr float PHASE_R    = 3.5f;    // ohms
constexpr float PHASE_L    = 0.0035f; // H (Ld = Lq)

// --- Control (PI pole placement) ---
constexpr float BANDWIDTH = 1000.0f;              // rad/s
constexpr float KP        = BANDWIDTH * PHASE_L;   // 3.5
constexpr float KI        = BANDWIDTH * PHASE_R;   // 3500

// --- Current sense (INA186A3, 15 mohm, REF 1.65V, 3.3V ADC) ---
constexpr float SHUNT_OHMS    = 0.015f;
constexpr float INA_GAIN      = 100.0f;
constexpr float ISENSE_REF_V  = 1.65f;
constexpr float ADC_VREF      = 3.3f;
constexpr float ADC_MAX       = 4095.0f;
constexpr float VOLTS_PER_AMP = SHUNT_OHMS * INA_GAIN; // 1.5
constexpr float IMAX_A        = 1.1f;                  // command clamp / sensor range

// --- PWM / loop ---
constexpr uint32_t PWM_HZ        = 24000;
constexpr int      ADC_OVERSAMPLE = 4;

// --- Bus voltage (future: from ADC) ---
constexpr float VBUS_V = 12.0f;

// --- GPIO map ---
constexpr uint32_t PIN_ENA_2 = 0,  PIN_PHA_2 = 1,  PIN_ENB_2 = 2,  PIN_PHB_2 = 3;
constexpr uint32_t PIN_ENA_1 = 4,  PIN_PHA_1 = 5,  PIN_ENB_1 = 6,  PIN_PHB_1 = 7;
constexpr uint32_t PIN_LED_R = 8,  PIN_LED_G = 9,  PIN_LED_B = 14;
constexpr uint32_t PIN_BTN_STOP = 19, PIN_BTN_GO = 20;
constexpr uint32_t PIN_NSLEEP_1 = 21, PIN_NSLEEP_2 = 22;
constexpr uint32_t PIN_SOB_1 = 26, PIN_SOA_1 = 27, PIN_SOB_2 = 28, PIN_SOA_2 = 29;
constexpr uint32_t ADC_SOB_1 = 0, ADC_SOA_1 = 1, ADC_SOB_2 = 2, ADC_SOA_2 = 3;
constexpr uint32_t PIN_LOOP_TIMING = 10; // SPI1 SCK; bringup only

} // namespace rotev
```

- [ ] **Step 6: Write Unity test skeleton `test/test_foc_math/test_main.cpp`**

```cpp
#include <unity.h>
void setUp() {}
void tearDown() {}
void test_placeholder() { TEST_ASSERT_TRUE(true); }
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_placeholder);
  return UNITY_END();
}
```

- [ ] **Step 7: Run native tests to verify the toolchain works**

Run: `pio test -e native`
Expected: PASS (1 test, `test_placeholder`).

- [ ] **Step 8: Commit**

```bash
git add platformio.ini library.json library.properties keywords.txt src/constants.h test/
git commit -m "chore: scaffold RotEv2 library, manifests, constants, native test env"
```

---

### Task 1: FOC math — Park & inverse-Park transforms

**Files:**
- Create: `src/foc_math.h`, `src/foc_math.cpp`
- Test: `test/test_foc_math/test_main.cpp` (extend)

**Interfaces:**
- Produces:
  - `struct DQ { float d, q; };`
  - `struct AB { float a, b; };`
  - `DQ park(AB i, float theta_e);` — rotates αβ measured current into dq using electrical angle.
  - `AB inversePark(float ud, float uq, float theta_e, float vbus);` — dq voltages → αβ phase duties in **[-1, 1]** (normalized by `vbus`, clamped).

- [ ] **Step 1: Write failing tests for park/inversePark**

```cpp
#include <unity.h>
#include <cmath>
#include "foc_math.h"
using namespace rotev;

void setUp() {} void tearDown() {}

void test_park_zero_angle_is_identity() {
  DQ dq = park({1.0f, 0.5f}, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, dq.d);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, dq.q);
}
void test_park_ninety_deg_rotates() {
  // theta=pi/2: d = a*cos+b*sin = b ; q = -a*sin+b*cos = -a
  DQ dq = park({1.0f, 0.0f}, (float)M_PI/2);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, dq.d);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, -1.0f, dq.q);
}
void test_inverse_park_zero_angle_normalizes_by_vbus() {
  AB ab = inversePark(6.0f, 0.0f, 0.0f, 12.0f); // ud=6V of 12V -> 0.5
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.5f, ab.a);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, ab.b);
}
void test_inverse_park_clamps_to_unit() {
  AB ab = inversePark(24.0f, 0.0f, 0.0f, 12.0f); // 2.0 -> clamp 1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, ab.a);
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_park_zero_angle_is_identity);
  RUN_TEST(test_park_ninety_deg_rotates);
  RUN_TEST(test_inverse_park_zero_angle_normalizes_by_vbus);
  RUN_TEST(test_inverse_park_clamps_to_unit);
  return UNITY_END();
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pio test -e native`
Expected: FAIL — `foc_math.h` not found / undefined references.

- [ ] **Step 3: Write `src/foc_math.h`**

```cpp
#pragma once
namespace rotev {

struct DQ { float d, q; };
struct AB { float a, b; };

// Park: measured alpha-beta current -> dq at electrical angle theta_e.
DQ park(AB i, float theta_e);

// Inverse Park: dq voltage command -> normalized alpha-beta phase duty in [-1,1].
AB inversePark(float ud, float uq, float theta_e, float vbus);

} // namespace rotev
```

- [ ] **Step 4: Write `src/foc_math.cpp`**

```cpp
#include "foc_math.h"
#include <cmath>

namespace rotev {

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

DQ park(AB i, float theta_e) {
  float c = cosf(theta_e), s = sinf(theta_e);
  return { i.a * c + i.b * s, -i.a * s + i.b * c };
}

AB inversePark(float ud, float uq, float theta_e, float vbus) {
  float c = cosf(theta_e), s = sinf(theta_e);
  float va = ud * c - uq * s;   // volts
  float vb = ud * s + uq * c;
  float inv = (vbus > 0.0f) ? 1.0f / vbus : 0.0f;
  return { clampf(va * inv, -1.0f, 1.0f), clampf(vb * inv, -1.0f, 1.0f) };
}

} // namespace rotev
```

- [ ] **Step 5: Add `foc_math.cpp` to the native build filter**

Confirm `platformio.ini` `[env:native]` has `build_src_filter = +<foc_math.cpp>` (from Task 0). No change needed if present.

- [ ] **Step 6: Run tests to verify they pass**

Run: `pio test -e native`
Expected: PASS (all park/inversePark tests).

- [ ] **Step 7: Commit**

```bash
git add src/foc_math.h src/foc_math.cpp test/
git commit -m "feat: park and inverse-park transforms with vbus normalization"
```

---

### Task 2: FOC math — PI controller with anti-windup

**Files:**
- Modify: `src/foc_math.h`, `src/foc_math.cpp`
- Test: `test/test_foc_math/test_main.cpp` (extend)

**Interfaces:**
- Produces:
  - `struct PIState { float integ; };`
  - `float piStep(PIState& s, float error, float kp, float ki, float dt, float out_limit);` — returns clamped output; integrator clamped to prevent windup (back-calculation: integrator not accumulated when output is saturated in the same direction).
  - `void piReset(PIState& s);`

- [ ] **Step 1: Write failing PI tests**

```cpp
void test_pi_proportional_only_first_step() {
  PIState s; piReset(s);
  // dt small so integral term tiny; kp=3.5, error=1 -> ~3.5 + ki*dt
  float out = piStep(s, 1.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 1000.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.5f + 3500.0f*(1.0f/24000.0f), out);
}
void test_pi_output_clamped_to_limit() {
  PIState s; piReset(s);
  float out = piStep(s, 100.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 5.0f, out);
}
void test_pi_antiwindup_stops_integrating_when_saturated() {
  PIState s; piReset(s);
  for (int i = 0; i < 1000; ++i) piStep(s, 100.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 5.0f);
  // After saturation, a single negative error should immediately reduce output
  float out = piStep(s, -100.0f, 3.5f, 3500.0f, 1.0f/24000.0f, 5.0f);
  TEST_ASSERT_TRUE(out < 5.0f);
}
void test_pi_reset_clears_integrator() {
  PIState s; piReset(s);
  piStep(s, 1.0f, 3.5f, 3500.0f, 0.1f, 1000.0f);
  piReset(s);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, s.integ);
}
```

Add the four `RUN_TEST(...)` lines to `main()`.

- [ ] **Step 2: Run to verify failure**

Run: `pio test -e native`
Expected: FAIL — `PIState`/`piStep`/`piReset` undefined.

- [ ] **Step 3: Extend `src/foc_math.h`**

```cpp
struct PIState { float integ; };
void  piReset(PIState& s);
float piStep(PIState& s, float error, float kp, float ki, float dt, float out_limit);
```

- [ ] **Step 4: Extend `src/foc_math.cpp`**

```cpp
void piReset(PIState& s) { s.integ = 0.0f; }

float piStep(PIState& s, float error, float kp, float ki, float dt, float out_limit) {
  float integ_next = s.integ + ki * error * dt;
  float unsat = kp * error + integ_next;
  float out = clampf(unsat, -out_limit, out_limit);
  // Anti-windup: only commit the integrator if we are not saturating further out.
  if (out == unsat) s.integ = integ_next;
  return out;
}
```

- [ ] **Step 5: Run to verify pass**

Run: `pio test -e native`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/foc_math.h src/foc_math.cpp test/
git commit -m "feat: PI controller with output clamp and anti-windup"
```

---

### Task 3: FOC math — electrical angle, ωe estimator, ADC scaling, LED duty

**Files:**
- Modify: `src/foc_math.h`, `src/foc_math.cpp`
- Test: `test/test_foc_math/test_main.cpp` (extend)

**Interfaces:**
- Produces:
  - `float electricalAngle(float theta_mech);` → `theta_mech * POLE_PAIRS`.
  - `struct OmegaEst { float prev_theta_e; float w_filt; bool primed; };`
  - `float omegaStep(OmegaEst& s, float theta_e, float dt, float alpha);` — returns low-pass-filtered electrical speed (rad/s); `alpha` is the LPF coefficient in (0,1].
  - `void omegaReset(OmegaEst& s);`
  - `float countsToAmps(uint16_t counts);` — `((counts/ADC_MAX)*ADC_VREF - ISENSE_REF_V) / VOLTS_PER_AMP`.
  - `float clampCurrent(float amps);` — clamp to ±`IMAX_A`.
  - `uint16_t ledDuty(uint8_t value, uint16_t top);` — active-low: returns `top - (value*top)/255`.

- [ ] **Step 1: Write failing tests**

```cpp
#include "constants.h"

void test_electrical_angle_scales_by_pole_pairs() {
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 50.0f, electricalAngle(1.0f));
}
void test_omega_estimator_constant_velocity() {
  OmegaEst s; omegaReset(s);
  float dt = 1.0f/12000.0f, w = 0.0f;
  float theta = 0.0f;
  for (int i = 0; i < 500; ++i) { theta += 100.0f*dt; w = omegaStep(s, theta, dt, 0.05f); }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100.0f, w); // converges toward 100 rad/s
}
void test_counts_to_amps_midscale_is_zero() {
  // 1.65V -> counts = 1.65/3.3*4095 = 2047.5
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, countsToAmps(2048));
}
void test_counts_to_amps_one_amp() {
  // V = 1.65 + 1.5 = 3.15 -> counts = 3.15/3.3*4095 = 3908.8
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, countsToAmps(3909));
}
void test_clamp_current_limits() {
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.1f, clampCurrent(5.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5, -1.1f, clampCurrent(-5.0f));
}
void test_led_duty_active_low() {
  TEST_ASSERT_EQUAL_UINT16(0, ledDuty(255, 3125));     // full on -> pin low
  TEST_ASSERT_EQUAL_UINT16(3125, ledDuty(0, 3125));    // off -> pin high
}
```

Add matching `RUN_TEST(...)` lines. Ensure `[env:native] build_flags` includes `-I src` (from Task 0) so `constants.h` resolves.

- [ ] **Step 2: Run to verify failure**

Run: `pio test -e native`
Expected: FAIL — new symbols undefined.

- [ ] **Step 3: Extend `src/foc_math.h`**

```cpp
#include <cstdint>

float electricalAngle(float theta_mech);

struct OmegaEst { float prev_theta_e; float w_filt; bool primed; };
void  omegaReset(OmegaEst& s);
float omegaStep(OmegaEst& s, float theta_e, float dt, float alpha);

float    countsToAmps(uint16_t counts);
float    clampCurrent(float amps);
uint16_t ledDuty(uint8_t value, uint16_t top);
```

- [ ] **Step 4: Extend `src/foc_math.cpp`**

```cpp
#include "constants.h"

float electricalAngle(float theta_mech) { return theta_mech * (float)POLE_PAIRS; }

void omegaReset(OmegaEst& s) { s.prev_theta_e = 0.0f; s.w_filt = 0.0f; s.primed = false; }

float omegaStep(OmegaEst& s, float theta_e, float dt, float alpha) {
  if (!s.primed) { s.prev_theta_e = theta_e; s.primed = true; return 0.0f; }
  float raw = (theta_e - s.prev_theta_e) / dt;
  s.prev_theta_e = theta_e;
  s.w_filt += alpha * (raw - s.w_filt);
  return s.w_filt;
}

float countsToAmps(uint16_t counts) {
  float v = ((float)counts / ADC_MAX) * ADC_VREF;
  return (v - ISENSE_REF_V) / VOLTS_PER_AMP;
}

float clampCurrent(float amps) { return clampf(amps, -IMAX_A, IMAX_A); }

uint16_t ledDuty(uint8_t value, uint16_t top) {
  uint32_t on = ((uint32_t)value * top) / 255u;   // desired brightness level
  return (uint16_t)(top - on);                      // active-low invert
}
```

- [ ] **Step 5: Run to verify pass**

Run: `pio test -e native`
Expected: PASS (all foc_math tests).

- [ ] **Step 6: Commit**

```bash
git add src/foc_math.h src/foc_math.cpp test/
git commit -m "feat: electrical angle, omega estimator, ADC scaling, active-low LED duty"
```

---

### Task 4: `hw` module — clock, pin init, nSLEEP

**Files:**
- Create: `src/hw.h`, `src/hw.cpp`

**Interfaces:**
- Consumes: `constants.h`.
- Produces:
  - `void hwInit();` — `set_sys_clock_khz(150000, true)` (overclock target validated later), init all GPIO directions/functions per the map, buttons with pull-downs, `nSLEEP` low.
  - `void hwSetNsleep(Motor m, bool on);` — drive `nSLEEP_1/2`.

> Validation for Tasks 4–11 is **compile + on-target bringup** (there is no host harness for register code). Each task ends by compiling against `env:rp2350` and, where a bringup sketch exists, flashing and observing.

- [ ] **Step 1: Write `src/hw.h`**

```cpp
#pragma once
#include "constants.h"
namespace rotev {
void hwInit();
void hwSetNsleep(Motor m, bool on);
}
```

- [ ] **Step 2: Write `src/hw.cpp`**

```cpp
#include "hw.h"
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
```

- [ ] **Step 3: Compile-check**

Run: `pio run -e rp2350`
Expected: SUCCESS (library compiles under the RP2350 env once a consuming sketch exists; if the env reports "no sources", this is exercised via the bringup project in Task 12 — proceed).

- [ ] **Step 4: Commit**

```bash
git add src/hw.h src/hw.cpp
git commit -m "feat: hw module - clock, pin init, nSLEEP control"
```

---

### Task 5: `led` module — active-low RGB PWM

**Files:**
- Create: `src/led.h`, `src/led.cpp`

**Interfaces:**
- Consumes: `constants.h`, `foc_math.h` (`ledDuty`).
- Produces:
  - `void ledInit();` — configure PWM on LED_R/G/B at ~1 kHz, start all off (pins high).
  - `void ledSet(uint8_t r, uint8_t g, uint8_t b);` — set duties via `ledDuty`.

- [ ] **Step 1: Write `src/led.h`**

```cpp
#pragma once
#include <cstdint>
namespace rotev {
void ledInit();
void ledSet(uint8_t r, uint8_t g, uint8_t b);
}
```

- [ ] **Step 2: Write `src/led.cpp`**

```cpp
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
```

- [ ] **Step 3: Compile-check + commit**

Run: `pio run -e rp2350` (or defer to Task 12 bringup build).
Expected: SUCCESS.

```bash
git add src/led.h src/led.cpp
git commit -m "feat: active-low RGB LED PWM module"
```

---

### Task 6: `button` module — debounced reads

**Files:**
- Create: `src/button.h`, `src/button.cpp`

**Interfaces:**
- Consumes: `constants.h`.
- Produces: `bool buttonRead(Button b);` — returns debounced pressed state (active-high). Debounce via a short sampled-agreement filter (poll-based, no ISR).

- [ ] **Step 1: Write `src/button.h`**

```cpp
#pragma once
#include "constants.h"
namespace rotev { bool buttonRead(Button b); }
```

- [ ] **Step 2: Write `src/button.cpp`**

```cpp
#include "button.h"
#include "hardware/gpio.h"

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
```

- [ ] **Step 3: Compile-check + commit**

```bash
git add src/button.h src/button.cpp
git commit -m "feat: debounced button reads (active-high, pull-down)"
```

---

### Task 7: `debug` module — loop-timing pin

**Files:**
- Create: `src/debug.h`, `src/debug.cpp`

**Interfaces:**
- Produces (all no-ops unless `ENABLE_LOOP_TIMING` defined):
  - `void debugTimingInit();`
  - `void debugTimingHigh();` / `void debugTimingLow();` — inline, GPIO fast set/clear on `PIN_LOOP_TIMING`.

- [ ] **Step 1: Write `src/debug.h`**

```cpp
#pragma once
#include "constants.h"
#include "hardware/gpio.h"
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
}
```

- [ ] **Step 2: Write `src/debug.cpp`**

```cpp
#include "debug.h"
// All functionality is header-inline; this TU keeps the module linkable.
```

- [ ] **Step 3: Commit**

```bash
git add src/debug.h src/debug.cpp
git commit -m "feat: loop-timing debug pin (guarded by ENABLE_LOOP_TIMING)"
```

---

### Task 8: `pwm` module — center-aligned 24 kHz + per-phase voltage

**Files:**
- Create: `src/pwm.h`, `src/pwm.cpp`

**Interfaces:**
- Consumes: `constants.h`.
- Produces:
  - `uint16_t pwmTop();` — the computed phase-correct TOP.
  - `void pwmInit();` — configure the 4 EN pins as phase-correct PWM @ 24 kHz sharing TOP; PH pins are plain GPIO outputs (set in `hw`). Returns with all duties 0.
  - `uint pwmMasterSlice();` — slice number whose wrap IRQ is the control tick.
  - `void pwmSetPhase(uint32_t en_pin, uint32_t ph_pin, float duty_signed);` — `duty_signed` in [-1,1]: sets `ph_pin` = sign, `en_pin` level = `|duty|*TOP`.

- [ ] **Step 1: Write `src/pwm.h`**

```cpp
#pragma once
#include <cstdint>
#include "constants.h"
namespace rotev {
uint16_t pwmTop();
void     pwmInit();
unsigned pwmMasterSlice();
void     pwmSetPhase(uint32_t en_pin, uint32_t ph_pin, float duty_signed);
}
```

- [ ] **Step 2: Write `src/pwm.cpp`**

```cpp
#include "pwm.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace rotev {

static uint16_t s_top = 0;

uint16_t pwmTop() { return s_top; }

static void cfgEn(uint32_t pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  unsigned slice = pwm_gpio_to_slice_num(pin);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_phase_correct(&c, true);
  pwm_config_set_wrap(&c, s_top);
  pwm_config_set_clkdiv(&c, 1.0f);
  pwm_init(slice, &c, true);
  pwm_set_gpio_level(pin, 0);
}

void pwmInit() {
  // phase-correct: f = f_sys / (2 * (TOP+1)) -> TOP = f_sys/(2*PWM_HZ) - 1
  s_top = (uint16_t)(clock_get_hz(clk_sys) / (2u * PWM_HZ) - 1u);
  cfgEn(PIN_ENA_1); cfgEn(PIN_ENB_1); cfgEn(PIN_ENA_2); cfgEn(PIN_ENB_2);
}

unsigned pwmMasterSlice() { return pwm_gpio_to_slice_num(PIN_ENA_1); }

void pwmSetPhase(uint32_t en_pin, uint32_t ph_pin, float duty_signed) {
  bool positive = duty_signed >= 0.0f;
  gpio_put(ph_pin, positive ? 1 : 0);
  float mag = positive ? duty_signed : -duty_signed;
  if (mag > 1.0f) mag = 1.0f;
  pwm_set_gpio_level(en_pin, (uint16_t)(mag * s_top));
}

} // namespace rotev
```

- [ ] **Step 3: Commit**

```bash
git add src/pwm.h src/pwm.cpp
git commit -m "feat: center-aligned 24kHz PWM with per-phase PH/EN voltage output"
```

---

### Task 9: `adc` module — oversampled 2-channel sampling

**Files:**
- Create: `src/adc.h`, `src/adc.cpp`

**Interfaces:**
- Consumes: `constants.h`, `foc_math.h` (`countsToAmps`).
- Produces:
  - `void adcInit();` — `adc_init()`, `adc_gpio_init` on 26–29.
  - `AB adcSampleMotor(Motor m);` — sample the motor's two channels `ADC_OVERSAMPLE` times round-robin, average, convert to amps. Returns `{Ia, Ib}` (phase A, phase B).

- [ ] **Step 1: Write `src/adc.h`**

```cpp
#pragma once
#include "constants.h"
#include "foc_math.h"   // AB
namespace rotev {
void adcInit();
AB   adcSampleMotor(Motor m);
}
```

- [ ] **Step 2: Write `src/adc.cpp`**

```cpp
#include "adc.h"
#include "hardware/adc.h"

namespace rotev {

void adcInit() {
  adc_init();
  adc_gpio_init(PIN_SOB_1); adc_gpio_init(PIN_SOA_1);
  adc_gpio_init(PIN_SOB_2); adc_gpio_init(PIN_SOA_2);
}

static uint16_t oversample(uint ch) {
  adc_select_input(ch);
  uint32_t acc = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; ++i) acc += adc_read();
  return (uint16_t)(acc / ADC_OVERSAMPLE);
}

AB adcSampleMotor(Motor m) {
  uint chA = (m == MOTOR_1) ? ADC_SOA_1 : ADC_SOA_2;
  uint chB = (m == MOTOR_1) ? ADC_SOB_1 : ADC_SOB_2;
  uint16_t a = oversample(chA);
  uint16_t b = oversample(chB);
  return { countsToAmps(a), countsToAmps(b) }; // AB.a = phase A current, AB.b = phase B
}

} // namespace rotev
```

- [ ] **Step 3: Commit**

```bash
git add src/adc.h src/adc.cpp
git commit -m "feat: oversampled 2-channel current sampling with counts->amps"
```

---

### Task 10: `foc` module — core1 launch, PWM-wrap ISR, control step

**Files:**
- Create: `src/foc.h`, `src/foc.cpp`

**Interfaces:**
- Consumes: `constants.h`, `foc_math.h`, `pwm.h`, `adc.h`, `debug.h`.
- Produces:
  - `void focStart();` — init PWM/ADC, launch core1, register + enable the master-slice wrap IRQ on core1.
  - `void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled);` — spinlock-protected publish from core0.
  - `AB focTelemetry(Motor m);` — spinlock-protected last measured phase currents.
  - Feature flags per motor for lag comp: `void focSetLagComp(bool on);` (global; default off until Phase 4).

- [ ] **Step 1: Write `src/foc.h`**

```cpp
#pragma once
#include "constants.h"
#include "foc_math.h"
namespace rotev {
void focStart();
void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled);
AB   focTelemetry(Motor m);
void focSetLagComp(bool on);
}
```

- [ ] **Step 2: Write `src/foc.cpp`**

```cpp
#include "foc.h"
#include "pwm.h"
#include "adc.h"
#include "hw.h"
#include "debug.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

namespace rotev {

struct Setpoint { float theta_mech; float iq_cmd; bool enabled; };

static volatile Setpoint s_sp[2]   = {{0,0,false},{0,0,false}};
static volatile AB       s_tel[2]  = {{0,0},{0,0}};
static PIState  s_pid[2], s_piq[2];
static OmegaEst s_omega[2];
static volatile bool s_lagcomp = false;
static spin_lock_t* s_lock;
static volatile int s_turn = 0; // 0 -> motor1, 1 -> motor2

// per-motor pin sets
static void phasePins(Motor m, uint32_t& enA, uint32_t& phA, uint32_t& enB, uint32_t& phB) {
  if (m == MOTOR_1) { enA=PIN_ENA_1; phA=PIN_PHA_1; enB=PIN_ENB_1; phB=PIN_PHB_1; }
  else              { enA=PIN_ENA_2; phA=PIN_PHA_2; enB=PIN_ENB_2; phB=PIN_PHB_2; }
}

static void controlStep(Motor m) {
  Setpoint sp;
  uint32_t irq = spin_lock_blocking(s_lock);
  sp = (Setpoint)s_sp[m];
  spin_unlock(s_lock, irq);

  uint32_t enA,phA,enB,phB; phasePins(m, enA,phA,enB,phB);

  if (!sp.enabled) {
    piReset(s_pid[m]); piReset(s_piq[m]); omegaReset(s_omega[m]);
    pwmSetPhase(enA, phA, 0.0f); pwmSetPhase(enB, phB, 0.0f);
    return;
  }

  const float dt = 1.0f / (PWM_HZ / 2.0f); // 12 kHz per motor

  AB i = adcSampleMotor(m);                       // measured phase currents (alpha=A, beta=B)
  float theta_e = electricalAngle(sp.theta_mech);
  float we = omegaStep(s_omega[m], theta_e, dt, 0.05f);

  DQ dq = park(i, theta_e);                        // open-loop position assumption
  float uq = piStep(s_piq[m], sp.iq_cmd - dq.q, KP, KI, dt, VBUS_V);
  float ud = piStep(s_pid[m], 0.0f      - dq.d, KP, KI, dt, VBUS_V);
  if (s_lagcomp) {
    uq += we * PHASE_L * dq.d;   // +we*Ld*Id
    ud -= we * PHASE_L * dq.q;   // -we*Lq*Iq
  }

  AB v = inversePark(ud, uq, theta_e, VBUS_V);     // normalized duties [-1,1]
  pwmSetPhase(enA, phA, v.a);
  pwmSetPhase(enB, phB, v.b);

  uint32_t irq2 = spin_lock_blocking(s_lock);
  s_tel[m] = i;
  spin_unlock(s_lock, irq2);
}

static void __not_in_flash_func(pwmWrapISR)() {
  debugTimingHigh();
  pwm_clear_irq(pwmMasterSlice());
  Motor m = (s_turn == 0) ? MOTOR_1 : MOTOR_2;
  controlStep(m);
  s_turn ^= 1;
  debugTimingLow();
}

static void core1Entry() {
  debugTimingInit();
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwmWrapISR);
  pwm_set_irq_enabled(pwmMasterSlice(), true);
  irq_set_enabled(PWM_IRQ_WRAP, true);   // serviced on core1 (this core)
  while (true) __wfi();
}

void focStart() {
  s_lock = spin_lock_init(spin_lock_claim_unused(true));
  pwmInit();
  adcInit();
  for (int m = 0; m < 2; ++m) { piReset(s_pid[m]); piReset(s_piq[m]); omegaReset(s_omega[m]); }
  multicore_launch_core1(core1Entry);
}

void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].iq_cmd = clampCurrent(iq_cmd);
  s_sp[m].enabled = enabled;
  spin_unlock(s_lock, irq);
}

AB focTelemetry(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  AB t = (AB)s_tel[m];
  spin_unlock(s_lock, irq);
  return t;
}

void focSetLagComp(bool on) { s_lagcomp = on; }

} // namespace rotev
```

- [ ] **Step 3: Commit**

```bash
git add src/foc.h src/foc.cpp
git commit -m "feat: core1 FOC loop - PWM-wrap ISR, alternating motors, PI + lag comp"
```

---

### Task 11: `rotev` public API

**Files:**
- Create: `src/rotev.h`, `src/rotev.cpp`

**Interfaces:**
- Consumes: all modules.
- Produces the public surface exactly as the spec:
  - `void begin();`
  - `void motorEnable(Motor m);` / `void motorDisable(Motor m);`
  - `void motorWrite(float theta_rad, float amps, Motor m);`
  - `void ledColor(uint8_t r, uint8_t g, uint8_t b);`
  - `bool buttonPressed(Button b);`
  - `float motorCurrentA(Motor m);` / `float motorCurrentB(Motor m);`

- [ ] **Step 1: Write `src/rotev.h`**

```cpp
#pragma once
#include "constants.h"
namespace rotev {

void  begin();
void  motorEnable(Motor m);
void  motorDisable(Motor m);
void  motorWrite(float theta_rad, float amps, Motor m);
void  ledColor(uint8_t r, uint8_t g, uint8_t b);
bool  buttonPressed(Button b);
float motorCurrentA(Motor m);
float motorCurrentB(Motor m);

} // namespace rotev
```

- [ ] **Step 2: Write `src/rotev.cpp`**

```cpp
#include "rotev.h"
#include "hw.h"
#include "led.h"
#include "button.h"
#include "foc.h"

namespace rotev {

static float s_theta[2] = {0,0};
static float s_iq[2]    = {0,0};
static bool  s_en[2]    = {false,false};

void begin() {
  hwInit();
  ledInit();
  focStart();
}

void motorEnable(Motor m) {
  hwSetNsleep(m, true);
  s_en[m] = true;
  focSetpoint(m, s_theta[m], s_iq[m], true);
}

void motorDisable(Motor m) {
  s_en[m] = false;
  focSetpoint(m, s_theta[m], 0.0f, false);
  hwSetNsleep(m, false);
}

void motorWrite(float theta_rad, float amps, Motor m) {
  s_theta[m] = theta_rad;
  s_iq[m] = amps;
  focSetpoint(m, theta_rad, amps, s_en[m]);
}

void ledColor(uint8_t r, uint8_t g, uint8_t b) { ledSet(r, g, b); }
bool buttonPressed(Button b) { return buttonRead(b); }
float motorCurrentA(Motor m) { return focTelemetry(m).a; }
float motorCurrentB(Motor m) { return focTelemetry(m).b; }

} // namespace rotev
```

- [ ] **Step 3: Commit**

```bash
git add src/rotev.h src/rotev.cpp
git commit -m "feat: public rotev:: API surface"
```

---

### Task 12: Bringup project scaffold + Phase 1 (basic hardware)

**Files:**
- Create: `bringup/platformio.ini`, `bringup/src/main.cpp`, `bringup/phase1_hw.h`
- Create/Modify: `docs/BRINGUP.md` (Phase 1 section)

**Interfaces:**
- Consumes: `rotev::begin/motorEnable/motorDisable/ledColor/buttonPressed/motorCurrentA/B`.

- [ ] **Step 1: Write `bringup/platformio.ini`**

```ini
[env:bringup]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = generic_rp2350
board_build.core = earlephilhower
framework = arduino
monitor_speed = 115200
build_flags = -std=gnu++17 -DENABLE_LOOP_TIMING -DPHASE=1
lib_deps = symlink://../
```

- [ ] **Step 2: Write `bringup/src/main.cpp`**

```cpp
#include <Arduino.h>
#if PHASE == 1
  #include "../phase1_hw.h"
#elif PHASE == 2
  #include "../phase2_openloop.h"
#elif PHASE == 3
  #include "../phase3_foc.h"
#elif PHASE == 4
  #include "../phase4_full.h"
#endif
```

- [ ] **Step 3: Write `bringup/phase1_hw.h`**

```cpp
#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);   // nSLEEP high; no motion commanded
  motorEnable(MOTOR_2);
}

void loop() {
  if (buttonPressed(BTN_GO))   ledColor(0, 255, 0);
  if (buttonPressed(BTN_STOP)) ledColor(255, 0, 0);
  Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
  Serial.print(motorCurrentB(MOTOR_1)); Serial.print(',');
  Serial.print(motorCurrentA(MOTOR_2)); Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_2));
  delay(50);
}
```

- [ ] **Step 4: Build the bringup project**

Run: `cd bringup && pio run -e bringup`
Expected: SUCCESS — this is the first full compile of the library through a consumer. Fix any compile errors surfaced here in the relevant `src/` module, re-commit those fixes.

- [ ] **Step 5: Flash and validate on hardware (document in BRINGUP.md)**

Run: `pio run -e bringup -t upload && pio device monitor`
Pass criteria (record in `docs/BRINGUP.md`): (a) all four current readings sit near 0 A with drivers idle (within a few mA of zero at the 1.65 V bias); (b) pressing GO turns LED green, STOP turns it red; (c) measure `nSLEEP_1/2` high with a meter after `motorEnable`; (d) with `ENABLE_LOOP_TIMING`, GPIO10 currently idle (ISR not yet ticking motion — expected).

- [ ] **Step 6: Write `docs/BRINGUP.md` Phase 1 section**

Document: wiring, how to select the phase (`-DPHASE=1`), the exact `pio` commands, the pass criteria above, and troubleshooting (reversed current sign → swap SOA/SOB interpretation; LED inverted → check `LED_ACTIVE_HIGH` assumption).

- [ ] **Step 7: Commit**

```bash
git add bringup/ docs/BRINGUP.md
git commit -m "feat: bringup project + Phase 1 basic-hardware sketch and docs"
```

---

### Task 13: Bringup Phase 2 — open-loop SVPWM (60 RPM, 0.1 A)

**Files:**
- Create: `bringup/phase2_openloop.h`
- Modify: `docs/BRINGUP.md` (Phase 2 section)

**Interfaces:**
- Consumes: `rotev` API. Open-loop = command a steadily advancing `theta` with a fixed small current setpoint; because PI drives measured→setpoint, a low current setpoint at a moving angle produces the open-loop sine currents.

> Note: true "open-loop voltage" mode is achieved by commanding a small `iq` (0.1 A) while advancing theta at 60 RPM; the current controller holds 0.1 A. This exercises the same path and yields the two-sine-wave result. If a pure voltage-open-loop is later desired, add a `focOpenLoopVoltage()` hook behind the same module boundary.

- [ ] **Step 1: Write `bringup/phase2_openloop.h`**

```cpp
#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// 60 RPM = 1 rev/s = 2*pi rad/s mechanical.
static float theta = 0.0f;
static uint32_t last_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  last_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  theta += 2.0f * PI * dt;                 // 60 RPM
  motorWrite(theta, 0.1f, MOTOR_1);        // 0.1 A phase current
  Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_1));
  delayMicroseconds(500);
}
```

- [ ] **Step 2: Build**

Run: `cd bringup && pio run -e bringup -o build_flags="-std=gnu++17 -DENABLE_LOOP_TIMING -DPHASE=2"` *(or edit `-DPHASE=2` in `platformio.ini`)*
Expected: SUCCESS.

- [ ] **Step 3: Flash + validate**

Pass criteria (record in BRINGUP.md): (a) Serial Plotter shows two ~90°-shifted sine waves at ~50 Hz electrical (60 RPM × 50 pole pairs / 60 = 50 Hz); (b) amplitude ≈ 0.1 A; (c) scope on GPIO10 (`LOOP_TIMING_PIN`) shows a stable pulse every ~41.6 µs (24 kHz) and pulse width < ~20 µs (loop fits budget); (d) motor spins smoothly at ~60 RPM.

- [ ] **Step 4: Write BRINGUP.md Phase 2 section**

Document commands, expected plot (screenshot description), electrical-frequency math, timing-pin interpretation, and failure modes (jerky motion → check theta wrap/dt; flat current → check PH direction logic / nSLEEP).

- [ ] **Step 5: Commit**

```bash
git add bringup/phase2_openloop.h docs/BRINGUP.md
git commit -m "feat: bringup Phase 2 open-loop sine + docs"
```

---

### Task 14: Bringup Phase 3 — closed-loop FOC (no lag comp)

**Files:**
- Create: `bringup/phase3_foc.h`
- Modify: `docs/BRINGUP.md` (Phase 3 section)

- [ ] **Step 1: Write `bringup/phase3_foc.h`**

```cpp
#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

static float theta = 0.0f;
static uint32_t last_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  focSetLagComp(false);        // Phase 3: no lag comp
  motorEnable(MOTOR_1);
  last_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f; last_us = now;
  theta += 2.0f * PI * dt;                 // constant velocity 60 RPM
  motorWrite(theta, 0.5f, MOTOR_1);        // higher torque current
  Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_1));
  delayMicroseconds(500);
}
```

`focSetLagComp` must be declared in `foc.h` and re-exported; add `void focSetLagComp(bool);` usage note to BRINGUP. (It is already public in `foc.h` from Task 10.)

- [ ] **Step 2: Build with `-DPHASE=3`, flash, validate**

Pass criteria: (a) motor holds constant velocity under light load; (b) currents remain sinusoidal and track the 0.5 A envelope; (c) loop-timing pulse still < budget; (d) commanding a step in current shows fast, stable settling (no sustained oscillation → PI gains sane).

- [ ] **Step 3: Write BRINGUP.md Phase 3 section**, including how to run an S-curve by ramping `theta` velocity in the sketch, and pass/fail.

- [ ] **Step 4: Commit**

```bash
git add bringup/phase3_foc.h docs/BRINGUP.md
git commit -m "feat: bringup Phase 3 closed-loop FOC + docs"
```

---

### Task 15: Bringup Phase 4 — full library (lag comp, S-curves to 100 rot / 300 RPM)

**Files:**
- Create: `bringup/phase4_full.h`
- Modify: `docs/BRINGUP.md` (Phase 4 section)

- [ ] **Step 1: Write `bringup/phase4_full.h`**

```cpp
#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Trapezoidal/S-curve position profile to 100 rotations, peak 300 RPM.
static const float TARGET_REV = 100.0f;
static const float VMAX = 300.0f/60.0f * 2.0f*PI;   // rad/s mechanical
static const float AMAX = VMAX / 0.5f;              // reach vmax in 0.5 s
static float theta = 0.0f, vel = 0.0f;
static uint32_t last_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  focSetLagComp(true);          // Phase 4: enable lag compensation
  motorEnable(MOTOR_1);
  last_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f; last_us = now;
  float target = TARGET_REV * 2.0f*PI;
  float remaining = target - theta;
  float vstop = sqrtf(2.0f*AMAX*fabsf(remaining));   // decel envelope
  float vcmd = (vel < VMAX) ? vel + AMAX*dt : VMAX;
  if (vcmd > vstop) vcmd = vstop;
  vel = vcmd;
  theta += vel*dt;
  if (theta > target) theta = target;
  motorWrite(theta, 0.8f, MOTOR_1);
  Serial.print(motorCurrentA(MOTOR_1)); Serial.print(',');
  Serial.println(motorCurrentB(MOTOR_1));
  delayMicroseconds(500);
}
```

- [ ] **Step 2: Build `-DPHASE=4`, flash, validate**

Pass criteria: (a) completes 100 rotations and stops on target; (b) sustains 300 RPM (= 250 Hz electrical) with sinusoidal currents; (c) with lag comp ON, Id stays near 0 at speed vs Phase 3 (verify by temporarily logging `park` d-axis — optional debug build); (d) loop-timing pulse width still < budget at 300 RPM.

- [ ] **Step 3: Write BRINGUP.md Phase 4 section** — full procedure, the lag-comp on/off comparison, 300 RPM electrical-frequency math (250 Hz), and final acceptance checklist.

- [ ] **Step 4: Commit**

```bash
git add bringup/phase4_full.h docs/BRINGUP.md
git commit -m "feat: bringup Phase 4 full library with lag comp + S-curve"
```

---

### Task 16: README, example sketch, keywords, final packaging

**Files:**
- Modify: `README.md`, `keywords.txt`
- Create: `examples/Basic/Basic.ino`

- [ ] **Step 1: Write `examples/Basic/Basic.ino`**

```cpp
#include <rotev.h>
using namespace rotev;

void setup() {
  begin();
  motorEnable(MOTOR_1);
  ledColor(0, 0, 255);
}

void loop() {
  static float theta = 0;
  theta += 0.01f;
  motorWrite(theta, 0.3f, MOTOR_1);   // advance position, 0.3 A
  delay(1);
}
```

- [ ] **Step 2: Write `keywords.txt`**

```
begin	KEYWORD2
motorEnable	KEYWORD2
motorDisable	KEYWORD2
motorWrite	KEYWORD2
ledColor	KEYWORD2
buttonPressed	KEYWORD2
motorCurrentA	KEYWORD2
motorCurrentB	KEYWORD2
MOTOR_1	LITERAL1
MOTOR_2	LITERAL1
BTN_STOP	LITERAL1
BTN_GO	LITERAL1
```

- [ ] **Step 3: Rewrite `README.md`**

Include, per spec: overview; **GPIO/bus map** (full table incl. SPI1 GPIO10–13, I2C0 GPIO16–17, both usable as plain GPIO); the three exposed functions (motor control, RGB LED, buttons) with signatures + examples; **dual-core contract** (library owns core1, no `setup1()/loop1()`); motor specs (14HS11-1004: 1.8°/step, R=3.5 Ω, L=3.5 mH); PI tuning (`kP=BW·L`, `kI=BW·R`, BW=1000 rad/s); lag-comp equations; current-sense range (±1.1 A) and clamp; the 12 V bus assumption + future-ADC note; a link to `docs/BRINGUP.md`.

- [ ] **Step 4: Compile the example under the library env**

Run: `cd bringup && pio run -e bringup` (still builds the library cleanly)
Expected: SUCCESS.

- [ ] **Step 5: Commit**

```bash
git add README.md keywords.txt examples/
git commit -m "docs: README with API, GPIO map, dual-core contract, motor/tuning specs; add example"
```

---

### Task 17: Finish the branch

- [ ] **Step 1: Run the full native test suite once more**

Run: `pio test -e native`
Expected: PASS (all foc_math tests).

- [ ] **Step 2: Confirm the bringup project builds all four phases**

Run for N in 1..4: `cd bringup && pio run -e bringup` with `-DPHASE=N`.
Expected: SUCCESS each.

- [ ] **Step 3: Use `superpowers:finishing-a-development-branch`** to decide merge/PR/cleanup for `feature/foc-library`.

---

## Self-Review

**Spec coverage:**
- Framework (arduino-pico + Pico SDK internals) → Task 0, all firmware tasks ✓
- Float + sinf/cosf → Task 1/`foc_math` ✓ (LUT fallback noted in spec; only if bringup fails timing — timing checked in Tasks 13–15 via `LOOP_TIMING_PIN`)
- Dual-core (FOC core1 / user core0) → Task 10 ✓; README contract → Task 16 ✓
- `rotev::` free-function API → Task 11 ✓
- ωe by differentiating commanded theta + LPF → Task 3 (`omegaStep`), used in Task 10 ✓
- INA186A3 100 V/V scaling, ±1.1 A clamp → Task 3 (`countsToAmps`, `clampCurrent`) ✓
- ADC oversampling default 4×, 2 channels/loop → Task 9 ✓
- Center-aligned 24 kHz PWM, 12 kHz alternation → Tasks 8, 10 ✓
- Park (open-loop position) / PI (kP=3.5,kI=3500) / lag comp / inverse-park with vbus boundary → Tasks 1–3, 10 ✓
- LED active-low → Task 3 (`ledDuty`), Task 5 ✓
- Buttons active-high pull-down debounced → Task 6 ✓
- `LOOP_TIMING_PIN` GPIO10 bringup-only behind `ENABLE_LOOP_TIMING` → Task 7, used in Tasks 12–15 ✓
- Bringup project isolated, 4 phases → Tasks 12–15 ✓
- README + docs/BRINGUP.md → Tasks 12–16 ✓
- CLAUDE.md workflow (tag, branch, per-phase validation) → Task 0 (tag/branch), Tasks 12–15 (validation), Task 17 (finish) ✓

**Placeholder scan:** No TBD/TODO; every code step has complete code; hardware-validation steps have explicit pass criteria. ✓

**Type consistency:** `AB`/`DQ`/`PIState`/`OmegaEst` defined in Task 1–3, consumed identically in Tasks 9–11. `park`/`inversePark`/`piStep`/`omegaStep`/`countsToAmps`/`clampCurrent`/`ledDuty`/`electricalAngle` signatures match across definer and callers. `focSetpoint/focTelemetry/focSetLagComp/focStart` consistent between `foc.h` (Task 10) and `rotev.cpp` (Task 11) and bringup sketches. ✓

**Known hardware-tuning risks (validated, not blocking the plan):** exact ADC sample instant within the center-aligned window and overclock ceiling are tuned during Tasks 13–15 using `LOOP_TIMING_PIN`; `ADC_OVERSAMPLE` drops to 2× if the 4× burst proves too wide. These are the spec's explicitly-open hardware assumptions.
