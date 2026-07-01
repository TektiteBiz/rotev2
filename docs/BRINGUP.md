# RotEv2 FOC Library — Board Bringup Guide

This document is the step-by-step operator guide for bringing up the RotEv2 stepper FOC board.
Follow the four phases in order. Every phase builds on the last. If hardware deviates, consult the
Tunable Knobs table — it is the only place you should be editing values during bringup.

---

## 1. How to Use This Guide

### 1.1 Prerequisites

- RP2350 board assembled, bus voltage at 12 V (VBUS), USB connected.
- PlatformIO installed (CLI or VS Code extension).
- Oscilloscope with at least two channels and at least 1 MHz bandwidth. Differential probes or
  a current-sense clamp are helpful for current waveforms.
- Serial terminal or Arduino Serial Plotter open at **115200 baud**.

### 1.2 Dual-Core Contract

The library owns **core1 exclusively**. The 24 kHz PWM wrap ISR (`pwmWrapISR`) is installed on
core1 and runs the FOC control loop there. Your sketch code — everything in `setup()` and `loop()`
— runs on core0. This split is mandatory:

- User sketches must **not** define `setup1()` or `loop1()`. Doing so will conflict with the
  library's `multicore_launch_core1` call and will produce undefined behavior.
- All cross-core communication between your `loop()` and the FOC loop is mediated by spin-locks
  inside the library. Never call FOC internals directly from setup1/loop1.

### 1.3 Flashing a Phase

All four environments live in the `bringup/` subdirectory. Flash any phase with:

```
cd bringup
pio run -e phase1 -t upload   # replace phase1 with phase2, phase3, or phase4
```

After upload, open the serial monitor (or Arduino Serial Plotter) in a second terminal:

```
pio device monitor
```

The monitor speed is fixed at **115200 baud** in `bringup/platformio.ini`.

### 1.4 Loop-Timing Pin (GPIO10)

All four `phase*` environments define `ENABLE_LOOP_TIMING`. This causes `src/debug.h` to drive
**GPIO10** (SPI1 SCK) high at the start of `pwmWrapISR` and low at the end. Attach an
oscilloscope probe to GPIO10 and trigger on it to measure the ISR duration.

Expected characteristics:
- Pulse period: ~41.6 µs — the ISR fires at 24 kHz and GPIO10 pulses on every call. Each
  motor is serviced on every other call (12 kHz per motor), but you will see one pulse per
  41.6 µs on the scope, not one per 83.3 µs.
- Pulse width: the time consumed by one `controlStep()` call, including ADC sampling and PWM
  update. With the default settings this should be well under 20 µs.

Scope GPIO10 alongside a phase EN pin and a current waveform. If the pulse width approaches or
exceeds half the PWM period, consult the `OVERSAMPLE` and `SYSCLK` knobs and the flash-jitter
note in Section 3.

### 1.5 The Golden Rule

During bringup, **only adjust values listed in the Tunable Knobs table (Section 3)**. Do not
rewrite library internals to chase a symptom. The table maps every observable failure mode to
the one or two knobs that govern it.

### 1.6 Behavior Caveat: motorEnable and the Stored Setpoint

`motorEnable(m)` re-applies the last commanded setpoint (theta, iq_cmd) that was written with
`motorWrite()`. If you call `motorEnable()` before any `motorWrite()`, the stored iq_cmd is 0 A
and the motor will be held in position but not spin. To ensure you start from rest, call
`motorWrite(theta, 0.0f, m)` to zero the current command **before** enabling:

```cpp
motorWrite(0.0f, 0.0f, MOTOR_1);  // zero setpoint first
motorEnable(MOTOR_1);              // then enable
```

### 1.7 Benign LED Boot Artifact

The LED may flash on for approximately 1 ms at power-up due to PWM init ordering. This is
harmless and disappears as soon as `ledInit()` completes.

---

## 2. Per-Phase Step-Through

### Phase 1 — Basic Hardware Verification (`phase1`)

**Purpose.** Confirm that the driver ICs, current sense amplifiers, buttons, and LED all work
before attempting any motor motion. No PWM is applied to the motor windings beyond what the
driver requires to acknowledge enable.

**Flash command.**

```
cd bringup && pio run -e phase1 -t upload && pio device monitor
```

**What to watch.**

Open the Arduino Serial Plotter. Four traces appear — `motorCurrentA(MOTOR_1)`,
`motorCurrentB(MOTOR_1)`, `motorCurrentA(MOTOR_2)`, `motorCurrentB(MOTOR_2)` — printed at 50 ms
intervals. Because no current command is applied, all four should hover near 0 A, with only the
small ADC offset bias visible (the INA186A3 reference is 1.65 V; the ADC reads this as 0 A).
With the motor unpowered the traces may wander by a few milliamps due to common-mode and thermal
noise; this is normal.

Press **GO** (GPIO20). The LED should turn green. Press **STOP** (GPIO19). The LED should turn red.

With a multimeter, measure `nSLEEP_1` (GPIO21) and `nSLEEP_2` (GPIO22). Both should be logic
HIGH (~3.3 V) after `motorEnable()` runs in `setup()`.

**Pass criteria.**

- All four current traces at idle: within ±few mA of 0 A.
- GO button → LED green; STOP button → LED red.
- `nSLEEP_1` and `nSLEEP_2` both measure logic HIGH after reset.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| LED shows wrong color (red for GO, green for STOP) | Swap the `ledColor()` calls in `phase1_hw.h`, or check button wiring |
| LED is always on, always off, or wrong color hue | `LED-POL` — see `ledDuty` in `src/foc_math.cpp` |
| Current traces are offset by ±0.1 A or more | `VREF` — check `ADC_VREF` in `src/constants.h`; verify 1.65 V on the INA186A3 REF pins |
| Current traces show wrong polarity (positive when negative expected) | `ISENSE-SIGN` — swap SOA/SOB in `src/adc.cpp` or negate the result |
| `nSLEEP` pins remain low | Driver IC power supply missing; check 12 V rail and level-shifter |

---

### Phase 2 — Open-Loop Sinusoidal Drive (`phase2`)

**Purpose.** Drive one motor open-loop at 60 RPM with a 0.1 A current command. Verify that the
Park/inverse-Park math and PWM generation produce the correct two-phase sinusoidal output, and
that the control ISR meets its timing budget.

**Flash command.**

```
cd bringup && pio run -e phase2 -t upload && pio device monitor
```

**What to watch.**

In the Serial Plotter you will see two traces: `motorCurrentA(MOTOR_1)` and
`motorCurrentB(MOTOR_1)`. At 60 RPM with 50 pole pairs, the electrical frequency is:

```
f_electrical = 60 RPM * 50 / 60 = 50 Hz
```

The two current traces should appear as sinusoids approximately 90° apart (alpha and beta phases),
running at 50 Hz, with amplitude near 0.1 A. The exact amplitude depends on motor back-EMF and
winding impedance at this operating point; values in the range 0.05–0.15 A are acceptable.

On the oscilloscope, monitor GPIO10. The timing pulses should arrive at approximately 41.6 µs
intervals (24 kHz) with each pulse width comfortably inside 20 µs. A pulse that is much wider, or
that varies widely from cycle to cycle, indicates the ISR is overrunning or encountering flash
cache misses (see the `FLASH-TIMING` note in Section 3 and the SYSCLK/OVERSAMPLE knobs).

Observe the motor physically. It should rotate smoothly and continuously at approximately 60 RPM.
If the motor jitters, oscillates, or stalls, begin with the `PH-DIR` knob.

**Pass criteria.**

- Two sinusoidal current traces at ~50 Hz, ~90° apart, amplitude ~0.1 A.
- GPIO10 pulse period ~41.6 µs; pulse width within budget (well under ~20 µs).
- Motor rotates smoothly at ~60 RPM.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| Motor does not rotate; current traces flat or near-zero | `PH-DIR` — sign in `src/pwm.cpp` `pwmSetPhase`; also verify driver nENABLE logic |
| Motor steps roughly or oscillates at low speed | `PROFILE-DT` — `delayMicroseconds(500)` in `phase2_openloop.h`; reduce if loop is too coarse |
| GPIO10 pulse too wide or jittery | `OVERSAMPLE` (drop to 2) or `SYSCLK` (raise to 200000); see `FLASH-TIMING` note |
| Motor spins the wrong direction | `PH-DIR` — negate `duty_signed` or swap EN/PH pins in `src/pwm.cpp` |
| Current amplitude much higher than 0.1 A | `ISENSE-SIGN` or `VREF` miscalibrated from Phase 1 |

---

### Phase 3 — Closed-Loop FOC (`phase3`)

**Purpose.** Close the current loop. The sketch commands a constant velocity of 60 RPM and a
0.5 A quadrature current. With the PI controller active, the d-axis current is regulated to zero
and the q-axis tracks the command. Lag compensation is explicitly disabled via `setLagComp(false)`
so that the baseline PI controller can be tuned without the feed-forward term.

**Flash command.**

```
cd bringup && pio run -e phase3 -t upload && pio device monitor
```

**What to watch.**

The Serial Plotter again shows the two phase currents. Because the closed-loop controller is now
regulating, the waveform shape will differ from Phase 2: the amplitude is set by the 0.5 A command
and the PI gains maintain it. Watch for:

- The traces should be smooth sinusoids at 50 Hz (60 RPM, 50 pole pairs) with amplitude consistent
  with 0.5 A command.
- There should be no sustained oscillation or growing envelope. A brief transient when the sketch
  first enables the motor is expected.
- The velocity should remain stable — no hunting or stuttering.

On GPIO10, confirm the timing budget is still met at this higher current command.

**Pass criteria.**

- Motor holds constant 60 RPM velocity without sustained oscillation.
- Phase current traces track the 0.5 A commanded envelope.
- Step response (enable → steady state) settles within a few electrical cycles.
- GPIO10 pulse width still within budget.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| Sustained oscillation or limit-cycling | `KP` / `KI` — lower KP first, then KI; or lower `BANDWIDTH` and recalculate both |
| Current tracks sluggishly, large steady-state error | `KP` / `KI` — raise, or raise `BANDWIDTH` |
| Noisy current signals causing instability | `OVERSAMPLE` — raise to 8 (watch timing budget); also check for ground loops on SOA/SOB |
| ADC sampling hits a switching edge (spikes on current traces) | `SAMPLE-INSTANT` — shift the ADC burst offset in `src/foc.cpp` `controlStep` toward the PWM center |
| Motor stalls under load | `IMAX_A` — current is being clamped; verify hardware limit before raising |

---

### Phase 4 — Full Library: Position Profile with Lag Compensation (`phase4`)

**Purpose.** Execute a trapezoidal position profile to 100 mechanical rotations at a peak velocity
of 300 RPM, then stop on target. Lag compensation is enabled via `setLagComp(true)`. At 300 RPM
the electrical frequency reaches:

```
f_electrical = 300 RPM * 50 / 60 = 250 Hz
```

This is the most demanding operating point. At 250 Hz electrical, the motor's inductive reactance
is significant and without the feed-forward decoupling term the d-axis current will grow, wasting
torque and increasing heating.

**Note on profile shape.** The Phase 4 motion profile is trapezoidal (linear acceleration and
deceleration ramps), not a true jerk-limited S-curve. The deceleration uses a velocity envelope
computed from `sqrt(2*AMAX*|remaining|)` to guarantee stop on target. Expect a brief current spike
at the decel-to-stop transition; this is a property of the trapezoidal profile, not a tuning issue.

**Flash command.**

```
cd bringup && pio run -e phase4 -t upload && pio device monitor
```

**What to watch.**

In the Serial Plotter, observe the current traces over the full 100-revolution profile. During the
acceleration ramp (reaching 300 RPM), the traces should increase in frequency to ~250 Hz. During
the constant-velocity segment, the 0.8 A commanded current should be tracked. As the motor
decelerates, frequency falls back. The motor should stop with minimal overshoot.

To verify lag compensation effectiveness, you can compare a run with `setLagComp(true)` (Phase 4
as shipped) against `setLagComp(false)` (temporarily): with lag comp ON, the d-axis current
(observable via a Park transform on the measured alpha/beta currents) should remain nearer to
zero at 300 RPM than with it off.

On GPIO10, the timing budget is tightest at 300 RPM because the ADC sampling and math consume a
fixed amount of time regardless of electrical frequency — monitor carefully that the pulse width
stays within budget throughout the acceleration profile.

**Pass criteria.**

- Motor completes exactly 100 rotations and stops on target (or within one mechanical degree).
- Peak velocity reaches ~300 RPM (250 Hz electrical).
- With lag comp ON, d-axis current remains nearer 0 at high speed than without it.
- GPIO10 pulse width within budget throughout the full profile.
- No overcurrent fault, driver nFAULT assertion, or thermal shutdown.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| D-axis current grows significantly at high speed | Confirm `setLagComp(true)` is in effect; check `PHASE_L` value in `src/constants.h` (0.0035 H) |
| Motor stops short of or past 100 rotations | Profile is integrating `theta`; verify `delay` / `dt` accuracy in loop; check for ISR overruns that slow the loop |
| Timing overrun at 300 RPM | `OVERSAMPLE` (drop to 2) or `SYSCLK` (raise the `set_sys_clock_khz` arg, e.g. 200000 = 200 MHz); see `FLASH-TIMING` note |
| Oscillation or instability at high speed | `KP` / `KI` — the plant changes at high electrical frequency; lower gains slightly |
| Hard stop / position error | `IMAX_A` saturating and not delivering enough torque; verify bus voltage and winding resistance |

---

## 3. Tunable Knobs Reference

These are the **only** values an operator should edit during bringup. Everything else in the
library is structural code that should not be changed without a design review.

| ID | Knob | File / Location | Default | When to Change |
|----|------|----------------|---------|---------------|
| `SYSCLK` | `set_sys_clock_khz(...)` in `hwInit()` | `src/hw.cpp` line 10 | `150000` (150 MHz) | Timing overrun at high speed — raise to e.g. 200000; re-verify clock stability and flash/SRAM access |
| `OVERSAMPLE` | `ADC_OVERSAMPLE` | `src/constants.h` line 30 | `4` | GPIO10 pulse too wide — drop to `2`; current waveform noisy — raise (watch timing budget) |
| `SAMPLE-INSTANT` | ADC burst placement in `controlStep` | `src/foc.cpp` `controlStep()` | Wrap-aligned (start of ISR) | Current spikes on scope indicate sampling lands on switching edge — shift the burst call toward PWM center |
| `KP` / `KI` | `KP`, `KI` (derived from `BANDWIDTH * PHASE_L/R`) | `src/constants.h` lines 16–17 | `3.5` / `3500` | Oscillation → lower; sluggish tracking → raise; adjust via `BANDWIDTH` and let the `constexpr` derivation recompute |
| `BANDWIDTH` | `BANDWIDTH` | `src/constants.h` line 15 | `1000` (rad/s) | Primary knob for PI response speed; change this and `KP`/`KI` follow automatically |
| `IMAX_A` | `IMAX_A` | `src/constants.h` line 26 | `1.1` A | Current sense range changes (different shunt or gain); also the command clamp |
| `ADC_VREF` | `ADC_VREF` | `src/constants.h` line 23 | `3.3` V | Measured ADC reference differs from 3.3 V (use a multimeter on the AVDD pin) |
| `VBUS_V` | `VBUS_V` | `src/constants.h` line 33 | `12.0` V | Different bus voltage; update before running if supply is not 12 V |
| `ISENSE-SIGN` | Swap `ADC_SOA_*`/`ADC_SOB_*` channels or negate | `src/adc.cpp` `adcSampleMotor()` | As wired (SOA = phase A, SOB = phase B) | Current polarity inverted vs. expectation from Phase 1 |
| `PH-DIR` | Sign of `duty_signed` in `pwmSetPhase` | `src/pwm.cpp` `pwmSetPhase()` | Positive duty → PH high | Motor spins opposite to commanded direction |
| `LED-POL` | `ledDuty` active-low inversion | `src/foc_math.cpp` `ledDuty()` | Active-low (inverted) | Board turns out to have common-anode LED or opposite polarity convention |
| `PROFILE-DT` | `delayMicroseconds(500)` in each phase sketch | `bringup/phase2_openloop.h`, `phase3_foc.h`, `phase4_full.h` | 500 µs | Command update rate — reduce for smoother profile; increase if CPU headroom is needed |
| `FLASH-TIMING` | Mark ISR chain `__not_in_flash_func` | `src/foc.cpp` `controlStep` and hot callees in `src/foc_math.cpp`, `src/pwm.cpp`, `src/adc.cpp` | Not applied | GPIO10 pulse width is wide or jittery even after OVERSAMPLE/SYSCLK tuning — this indicates XIP flash-cache misses adding latency. Apply `__not_in_flash_func` to `controlStep` and the functions it calls (`adcSampleMotor`, `pwmSetPhase`, `piStep`, `park`, `inversePark`). This is a structural change, not a constant, so verify by re-scoping GPIO10 after applying it. |

### Notes on the Current Sense Range

The INA186A3 has a gain of 100 and the shunt is 15 mΩ, giving 1.5 V/A sensitivity. The ADC
reference is 3.3 V and the INA output is biased at 1.65 V (ISENSE_REF_V), so the usable range is
approximately ±1.1 A. Commands above ±1.1 A are silently clamped. If you need higher current range
you must change the shunt resistor and update `SHUNT_OHMS`, `IMAX_A`, and `ISENSE_REF_V` in
`src/constants.h`.

### Note on Flash-Cache Jitter (FLASH-TIMING)

The 12 kHz control ISR chain (`pwmWrapISR` → `controlStep` → `adcSampleMotor`, `park`,
`piStep`, `inversePark`, `pwmSetPhase`) is not currently marked `__not_in_flash_func`. The outer
ISR wrapper `pwmWrapISR` is marked, but `controlStep` and its callees are not. On the RP2350 with
XIP flash, cache misses can add variable latency (several hundred nanoseconds per miss). If the
GPIO10 scope trace shows pulse widths that are wider than expected or vary cycle-to-cycle near
the PWM center point, apply `__not_in_flash_func` to the entire hot path. This ensures those
functions are placed in SRAM (or flash cache-locked region) at link time and removes jitter. Re-
scope GPIO10 after applying to confirm improvement.

---

## 4. Final Acceptance Checklist

Run through these checkboxes in order after all four phases pass. Do not merge the feature branch
until all boxes are checked.

### Hardware pass criteria

- [ ] **Phase 1:** All four idle current traces within ±few mA of 0 A; GO→green, STOP→red;
      nSLEEP_1 and nSLEEP_2 both measure logic HIGH.
- [ ] **Phase 2:** Two sinusoidal currents at ~50 Hz electrical (60 RPM × 50/60), ~90° apart,
      amplitude ~0.1 A; motor rotates smoothly at 60 RPM; GPIO10 pulse period ~41.6 µs, width
      within budget.
- [ ] **Phase 3:** Closed-loop velocity holds at 60 RPM; current tracks 0.5 A envelope; no
      sustained oscillation; GPIO10 still within budget.
- [ ] **Phase 4:** Trapezoidal profile completes 100 rotations and stops on target; 300 RPM
      sustained (250 Hz electrical); with lag comp ON, d-axis current stays nearer 0 than OFF;
      GPIO10 within budget throughout full profile.

### Build and test verification

- [ ] `pio test -e native` passes green (all unit tests pass).
- [ ] All four bringup environments compile without warning:
  ```
  cd bringup && pio run -e phase1 -e phase2 -e phase3 -e phase4
  ```
- [ ] Flash and RAM usage remain within comfortable margins (current builds: RAM ~1.9%, Flash ~0.4%
      of RP2350 totals — well clear of any limit).

### Documentation and wiring

- [ ] `README.md` bus map matches physical wiring (GPIO assignments, nSLEEP, SOA/SOB).

---

## 5. Finalize the Library (After All Phases Pass)

This section is the operator's last step once bringup is complete. It makes the repository tree
read as a clean, publishable first-pass implementation.

### 5.1 Resolve the Trig Path

The shipped library uses `sinf`/`cosf` throughout `src/foc_math.cpp` (in `park` and
`inversePark`). If GPIO10 never showed a timing overrun at any phase — the expected outcome given
the build sizes — no look-up table (LUT) was ever needed, and there is nothing to resolve.

If you were forced to add a sin/cos LUT to meet timing at high speed, you must delete the
`sinf`/`cosf` path and leave exactly one implementation. Never ship both. Audit:
```
grep -r "sinf\|sinLUT\|cosf\|cosLUT" src/
```
Exactly one implementation family should appear.

### 5.2 Delete Diagnostic Additions

Remove any temporary instrumentation added while chasing a bringup issue:

- `Serial.print` calls beyond the intended per-phase telemetry in the `phase*.h` sketches.
- Dead code branches, `#if 0` blocks, commented-out experiments.
- Any probe pin toggles beyond the `ENABLE_LOOP_TIMING` path (which stays).

### 5.3 Revert or Document Tuned Knobs

Any knob value you changed from its default during bringup must be either:

- Reverted to the default if you changed it temporarily to diagnose and the original value is
  correct for production, or
- Left at the new value with a one-line comment in the source explaining why (e.g.,
  `// raised from 150000 to 200000: required for <300RPM timing budget on this PCB rev`).

### 5.4 Production Build Flag

Confirm that `ENABLE_LOOP_TIMING` is defined **only** in the bringup environments
(`bringup/platformio.ini`). It must not appear in any consumer project's build flags. When
`ENABLE_LOOP_TIMING` is absent, GPIO10 (SPI1 SCK) is released for use by the application. Verify:
```
grep -r "ENABLE_LOOP_TIMING" .
```
It should appear only in `bringup/platformio.ini` and `src/debug.h` (the guard). No consumer
`platformio.ini` or user sketch should define it.

### 5.5 Re-verify After Edits

After completing all cleanup steps:

```
pio test -e native
cd bringup && pio run -e phase1 -e phase2 -e phase3 -e phase4
```

Both must succeed without error. If a phase sketch fails to compile after diagnostic cleanup,
the removal went too far — restore from git and retry.

### 5.6 Finish the Branch

Once all verifications pass:

```
git add -p                  # interactive: opens a patch selector to stage only intentional changes
git commit -m "feat: finalize foc-library after bringup"
```

Then use the `superpowers:finishing-a-development-branch` skill to merge or open a pull request
for `feature/foc-library` into `main`.

---

*End of bringup guide. Once Phase 4 passes and this checklist is complete, the library is ready
for integration.*
