# RotEv2: Buzzer, ADS1015 ADC, GPIO Renumbering — Design Spec

**Date:** 2026-07-13
**Status:** Approved (brainstorming)
**Source PRD:** `docs/PRD.md` (updated 2026-07-13: buzzer, ADS1015 ADC, GPIO renumbering)
**Supersedes/extends:** `docs/superpowers/specs/2026-06-30-rotev2-foc-library-design.md`

## Overview

The PRD picked up three changes since the original FOC library design was implemented:

1. A passive piezo buzzer on GPIO4 (user on/off + frequency control).
2. An ADS1015 external I2C ADC (bus-voltage sensing for FOC inverse-park, plus
   3 user-facing channels).
3. GPIO renumbering: buttons/nSLEEP shifted by one pin, I2C1 (GPIO18/19) added
   for the ADC. Motor PWM pin naming/numbering (GPIO0-3) is unaffected — already
   correct in the current codebase.

This spec covers incorporating all three into the existing library structure
(`src/adc.cpp` = internal phase-current ADC stays as-is; a new `adc_ext.*`
module handles the ADS1015).

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| GPIO constants | Update `constants.h` only | No other file hardcodes raw pin numbers (verified). Mechanical change. |
| Buzzer PWM | Dedicated slice on GPIO4 (slice 2) | No aliasing with motor (slices 0/1) or LED (slices 4/7) slices. |
| ADS1015 driver | Single-shot mode, raw `hardware/i2c.h` on I2C1 @ 400kHz | Matches SDK-direct style used elsewhere in the codebase (no Arduino Wire). Single-shot gives explicit per-channel control needed for round-robin. |
| ADS1015 scheduling | Automatic background timer, not user-pumped | I2C1 is a dedicated bus (not user-facing — that's I2C0 on GPIO16/17), so there's no contention risk in making this fully automatic. Removes a footgun where a user forgets to pump a manual function and gets stale/wrong bus voltage feeding FOC. |
| Round-robin weighting | AIN0 (bus voltage): 2 of every 3 slots; AIN1/2/3 share the remaining slot in rotation | AIN0 effective rate ≈1.2-1.5kHz (meets "~1kHz" spec); user channels ≈150-250Hz each (ample for display/telemetry). |
| Vbus → FOC handoff | Same spinlock pattern as existing setpoint/telemetry cross-core state | Reuses proven mechanism; `controlStep` reads live `s_vbus` instead of constant `VBUS_V`. |
| Vbus startup fallback | Seed cache with nominal `VBUS_V` constant before first real sample | No special-cased startup logic needed in FOC; degrades gracefully. |
| User ADC API | `float adcRead(AdcChannel ch)`, returns cached volts, non-blocking | Matches existing telemetry-getter pattern (`motorCurrentA/B`). |
| Bus voltage API | `float busVoltage()` also exposed publicly | Needed by Phase 1 bringup to display/compare Vbus against a multimeter reading; also generally useful telemetry for any user sketch. |
| Buzzer API | `buzzerOn(uint16_t freq_hz)` / `buzzerOff()` | Frequency clamped [1000,4000]Hz; calling `buzzerOn` again while sounding retunes on the fly. |

## Architecture

### New/changed modules
- `constants.h` — updated pin numbers (`PIN_BTN_STOP=20, PIN_BTN_GO=21,
  PIN_NSLEEP_1=22, PIN_NSLEEP_2=23`), new `PIN_I2C1_SDA=18, PIN_I2C1_SCL=19,
  PIN_BUZZ=4`, new `AdcChannel` enum (`ADC_AIN1..3`).
- `src/buzz.h` / `src/buzz.cpp` (new) — PWM slice init/frequency retuning for GPIO4.
- `src/adc_ext.h` / `src/adc_ext.cpp` (new) — ADS1015 driver: init, background
  alarm callback + state machine, cached-value getters (`adcExtVbus()`,
  `adcExtUser(AdcChannel)`).
- `src/foc.cpp` — `controlStep` reads `adcExtVbus()` instead of the `VBUS_V`
  constant when computing `inversePark`/PI clamps. `VBUS_V` in `constants.h`
  becomes only the startup-fallback nominal value.
- `src/rotev.h` / `src/rotev.cpp` — new public functions: `buzzerOn`,
  `buzzerOff`, `adcRead`.
- `README.md` — updated GPIO/bus table, new Buzzer section, new ADC/AIN section.

### ADS1015 driver detail
- I2C1 @ 400kHz, raw `hardware/i2c.h` calls (`i2c_write_blocking`,
  `i2c_read_blocking` — each call is a few bytes, ~100-150µs, so blocking calls
  from within the alarm callback are acceptable and simple).
- Single-shot config per conversion: mux = target channel, PGA = ±4.096V FS
  (covers the 0-3.3V range of both the bus-voltage divider output and
  assumed user-signal range), data rate = 3300 SPS (fastest, ~303µs conversion).
- State machine (driven by a repeating hardware alarm on core0, registered in
  `adcExtInit()` called from `rotev::begin()`):
  - `START_CONV`: write 3-byte config register selecting the next channel in
    the weighted round-robin; record tick count.
  - `READ_RESULT`: once enough ticks have elapsed (≥ conversion time), read the
    2-byte conversion register, convert counts→volts, store into the
    appropriate cached slot (`s_vbus` for AIN0, `s_user[ch]` for AIN1-3) under
    the existing spinlock, advance round-robin, transition back to `START_CONV`.
  - Each alarm tick performs at most one of these steps — bounded, non-blocking
    from the caller's perspective (the alarm fires independently of user code).
- Round-robin sequence: `AIN0, AIN1, AIN0, AIN2, AIN0, AIN3, ...` (AIN0 every
  other slot = 2 of every 3 total slots across the 3-user-channel cycle).
- Counts→volts: `V = counts * (4.096 / 2048)` (single-ended 12-bit, ±4.096V FS
  in the ADS1015's 12-bit-left-justified single-ended convention).
- Bus voltage: `Vbus = adcExtVbus_divider_out * (7.3+2.2)/2.2` (undo the
  voltage divider) before caching into `s_vbus`.

### FOC integration
`controlStep()` (in `foc.cpp`) currently uses the compile-time `VBUS_V`
constant everywhere a bus voltage is needed (PI output clamps, `inversePark`
call). This changes to read `adcExtVbus()` once at the top of each
`controlStep()` invocation (a plain spinlock-guarded float read, same cost
profile as the existing setpoint read) and thread that value through instead
of `VBUS_V`. `VBUS_V` remains defined as the pre-ADC-sample fallback default.

## Public API additions (`namespace rotev`)
```cpp
void buzzerOn(uint16_t freq_hz);   // clamped to [1000, 4000] Hz, 50% duty
void buzzerOff();
float adcRead(AdcChannel ch);      // ADC_AIN1/2/3, last cached sample in volts, non-blocking
float busVoltage();                // last cached Vbus sample in volts, non-blocking
```
`AdcChannel` is a new enum in `constants.h`.

## Testing
- Buzzer: bringup smoke test (audible tone, frequency sweep) — no host-testable
  logic beyond clamping, which can get a small native unit test alongside the
  existing `test_foc_math` target.
- ADS1015 counts→volts and divider-undo math: pure functions, host-testable
  the same way as `foc_math`.
- Vbus feeding into FOC: validated on-hardware during bringup (compare
  measured Vbus via `adcRead`-style debug print against a multimeter reading
  of the actual bus rail).

## Documentation Deliverables
- `README.md` — updated GPIO/pin table (new button/nSLEEP/I2C1 numbers),
  new **Buzzer** section (range, API, 50% duty note), new **ADC** section
  documenting `adcRead()`, the 3 user channels, sample-rate expectations, and
  that bus-voltage sensing is automatic/internal (no user action needed).

## Bringup Impact (Phase 1)

Phase 1 (`bringup/phase1_hw.h`) is the first sketch to exercise the new hardware, so it gains two
additions:

- **Buzzer smoke test on GO:** pressing GO (in addition to turning the LED green) plays a short
  fixed tune via `buzzerOn`/`buzzerOff` calls (a few notes within the 1-4kHz range, each held
  briefly then silenced) before returning to the idle loop. This exercises the buzzer PWM path and
  gives an audible pass/fail signal without needing a scope.
- **Bus voltage telemetry:** the periodic serial print (alongside the four `motorCurrentA/B`
  traces) adds a fifth value, `rotev::busVoltage()`, so the operator can visually confirm it
  against a multimeter reading of the 12V rail.

`docs/BRINGUP.md` Phase 1 section (What to watch / Pass criteria / Failure modes table) is updated
to describe both additions.

## Open Assumptions to Confirm on Hardware
- ADS1015 I2C1 bus has adequate pull-ups on the PCB (dedicated bus, assumed
  fixed near the ADC — same assumption class as the existing I2C0 pull-up note
  in the PRD).
- PGA ±4.096V FS is adequate headroom for the bus-voltage divider output across
  the expected Vbus range; revisit if bringup shows clipping or poor resolution.
