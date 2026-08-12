# RotEv2 FOC Library — Board Bringup Guide

This document is the step-by-step operator guide for bringing up the RotEv2 stepper FOC board.
Follow the phases in order — `phase1`, `phase1b`, `phase1c`, `phase2`, `phase3`, `phase4`, and the
`phase5` characterization run. Every phase builds on the last. If hardware deviates, consult the
Tunable Knobs table — it is the only place you should be editing values during bringup.

Phases 1b, 1c, 2 and 5 drive the inverter open-loop or read raw axis telemetry, which the public
API deliberately does not expose. They include `rotev_internal.h` for those primitives
(`motorSetVoltage`, `motorSetVoltageAB`, `motorSetVelocity`, `motorCurrent*`, `motorVoltage*`).
That header is for bringup only — application code uses `rotev.h` and nothing else.

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

All phase environments live in the `bringup/` subdirectory. Flash any phase with:

```
cd bringup
pio run -e phase1 -t upload   # or phase1b, phase1c, phase2, phase3, phase4, phase5
```

After upload, open the serial monitor (or Arduino Serial Plotter) in a second terminal:

```
pio device monitor
```

The monitor speed is fixed at **115200 baud** in `bringup/platformio.ini`.

### 1.4 Loop-Timing Pin (GPIO10)

The `phase1`, `phase2`, `phase3` and `phase4` environments define `ENABLE_LOOP_TIMING`. This causes `src/debug.h` to drive
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

### 1.6 Behavior Caveat: motorEnable and the Stored Profile

`motorEnable(m)` releases nSLEEP and lets the axis drive current; it commands no *motion* of its
own, but it does command a **hold**. An axis that has never been given a profile is energised at
standstill and resists being turned by hand — expect the shaft to feel locked, and expect the
first enable to snap the rotor into alignment with electrical angle 0, since with no position
sensor the commanded angle starts at 0 regardless of where the shaft actually is. A free shaft
after `motorEnable()` means the bridge is not driving; treat it as a fault, not as normal idle.

The order of `motorEnable()` and `motorSetProfile()` does not matter: a profile set on a disabled
axis is stored and its clock only starts once the axis is enabled, and a profile set on an axis
already holding replaces the hold.

Profiles are **relative** — the distance is measured from wherever the axis is when
`motorSetProfile()` is called, and the profile clock restarts at 0. Issuing a new profile mid-move
abandons the old one from the current position rather than queueing behind it.

Current is fixed at `MOTOR_AMPS` (0.8 A) for every closed-loop move and is not a knob. The
open-loop bringup phases command a voltage instead, so they set their own operating current
through Ohm's law.

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

Open the Arduino Serial Plotter. Five traces appear — `motorCurrentA(MOTOR_1)`,
`motorCurrentB(MOTOR_1)`, `motorCurrentA(MOTOR_2)`, `motorCurrentB(MOTOR_2)`, and
`busVoltage()` — printed at 50 ms intervals. Because the sketch commands 0 V explicitly after
enabling (`motorEnable()` would otherwise hold position at `MOTOR_AMPS`, which this phase must not
do with the loop still unverified), all four current traces should hover near 0 A, with only the
small ADC offset bias visible (the INA181A2
reference is 1.65 V; the ADC reads this as 0 A). With the motor unpowered the traces may wander by
a few milliamps due to common-mode and thermal noise; this is normal. `busVoltage()` should read
close to the actual 12 V rail (measure with a multimeter and compare).

Press **GO** (GPIO21). The LED should turn green, and the buzzer should play a short fixed tune
(a few notes, each within 1-4kHz, held briefly then silenced) via `buzzerOn`/`buzzerOff` — this is
the audible smoke test for the buzzer PWM path. Press **STOP** (GPIO20). The LED should turn red.

With a multimeter, measure `nSLEEP_1` (GPIO22) and `nSLEEP_2` (GPIO23). Both should be logic
HIGH (~3.3 V) after `motorEnable()` runs in `setup()`.

**Pass criteria.**

- All four current traces at idle: within ±few mA of 0 A.
- `busVoltage()` reads within a reasonable tolerance (e.g. ±0.5 V) of the multimeter-measured rail.
- GO button → LED green + audible tune; STOP button → LED red.
- `nSLEEP_1` and `nSLEEP_2` both measure logic HIGH after reset.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| LED shows wrong color (red for GO, green for STOP) | Swap the `ledColor()` calls in `phase1_hw.h`, or check button wiring |
| LED is always on, always off, or wrong color hue | `LED-POL` — see `ledDuty` in `src/foc_math.cpp` |
| Current traces are offset by ±0.1 A or more | `VREF` — check `ADC_VREF` in `src/constants.h`; verify 1.65 V on the INA181A2 REF pins |
| Current traces show wrong polarity (positive when negative expected) | `ISENSE-SIGN` — swap SOA/SOB in `src/adc.cpp` or negate the result |
| `nSLEEP` pins remain low | Driver IC power supply missing; check 12 V rail and level-shifter |
| No tune plays on GO, or tune is inaudible/distorted | Check `PIN_BUZZ` (GPIO4) wiring; verify buzzer frequency is within its 1-4kHz rated range |
| `busVoltage()` reads 0, wildly wrong, or never updates | Check I2C1 (GPIO18/19) wiring/pull-ups to the ADS1015; verify the divider resistors (7.3k/2.2k) |

---

### Phase 1b — Open-Loop Voltage Spin (`phase1b`)

**Purpose.** First motion, with the PI bypassed. `motorSetVoltage(MOTOR_1, theta, UQ_VOLTS)`
applies a q-axis voltage directly through the inverse Park at a slowly rotating angle (5 RPM), so
the inverter, ADC timing and telemetry all run normally while nothing can wind up or saturate.
This is where current-sense sign, amplitude and channel assignment get confirmed before the loop
is closed.

**Flash command.**

```
cd bringup && pio run -e phase1b -t upload && pio device monitor
```

**What to watch.** Two traces, `sensA` and `sensB`: sinusoids ~90° apart with amplitude near
`UQ_VOLTS / PHASE_R`. Wrong polarity versus the commanded direction is the `ISENSE-SIGN` knob
(`ISENSE_SCALE_A` / `ISENSE_SCALE_B` in `src/constants.h`) — invisible open-loop but fatal
closed-loop, so fix it here.

---

### Phase 1c — Stationary DC Current (`phase1c`)

**Purpose.** The same open-loop voltage path as 1b, but at a FIXED electrical angle, so the whole
commanded voltage lands on phase A instead of sweeping across A/B. At DC the winding is purely
resistive, so `V = I * R` predicts the current exactly and the sense scale can be checked against
a number rather than a shape.

**Flash command.**

```
cd bringup && pio run -e phase1c -t upload && pio device monitor
```

**What to watch.** `sensA` should sit at `UQ_VOLTS / PHASE_R` (steady DC, not a sinusoid) and
`sensB` near zero. A scale error here is the `ISENSE-CAL` knob.

---

### Phase 2 — Open-Loop Sinusoidal Drive (`phase2`)

**Purpose.** Drive one motor open-loop at 60 RPM, still on `motorSetVoltage` — a fixed VOLTAGE
chosen so the resulting phase current is the phase's `TARGET_AMPS`, not a current command. Verify
that the Park/inverse-Park math and PWM generation produce the correct two-phase sinusoidal output
at speed, and that the control ISR meets its timing budget.

At 50 Hz electrical the inductive reactance is no longer negligible next to the winding
resistance, so the voltage is sized from `|Z| = sqrt(R^2 + (we*L)^2)`, not from R alone — the
sketch shows the arithmetic. The mechanical speed is also ramped from zero over `RAMP_TIME_S`: a
stepper rotor cannot snap to 50 Hz electrical from standstill, the field would outrun its pull-in
torque and it would buzz in place.

**Flash command.**

```
cd bringup && pio run -e phase2 -t upload && pio device monitor
```

**What to watch.**

In the Serial Plotter you will see two traces, `sensA` and `sensB`. At 60 RPM with 50 pole pairs,
the electrical frequency is:

```
f_electrical = 60 RPM * 50 / 60 = 50 Hz
```

The two current traces should appear as sinusoids approximately 90° apart (alpha and beta phases),
running at 50 Hz, with amplitude near the sketch's `TARGET_AMPS`. The exact amplitude depends on
motor back-EMF and winding impedance at this operating point; ±30% is acceptable, since the
voltage is computed from nominal R and L and nothing is regulating it.

On the oscilloscope, monitor GPIO10. The timing pulses should arrive at approximately 41.6 µs
intervals (24 kHz) with each pulse width comfortably inside 20 µs. A pulse that is much wider, or
that varies widely from cycle to cycle, indicates the ISR is overrunning or encountering flash
cache misses (see the `FLASH-TIMING` note in Section 3 and the SYSCLK/OVERSAMPLE knobs).

Observe the motor physically. It should rotate smoothly and continuously at approximately 60 RPM.
If the motor jitters, oscillates, or stalls, begin with the `PH-DIR` knob.

**Pass criteria.**

- Two sinusoidal current traces at ~50 Hz, ~90° apart, amplitude near `TARGET_AMPS`.
- GPIO10 pulse period ~41.6 µs; pulse width within budget (well under ~20 µs).
- Motor rotates smoothly at ~60 RPM.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| Motor does not rotate; current traces flat or near-zero | `PH-DIR` — sign in `src/pwm.cpp` `pwmSetPhase`; also verify driver nENABLE logic |
| Motor buzzes in place instead of turning | `RAMP_TIME_S` in `phase2_openloop.h` — too short and the field outruns pull-in torque; `TARGET_AMPS` too low to beat detent torque |
| Motor steps roughly or oscillates at low speed | `CMD-DT` — `delayMicroseconds(500)` in `phase2_openloop.h`; reduce if the commanded angle is too coarse |
| GPIO10 pulse too wide or jittery | `OVERSAMPLE` (drop to 2) or `SYSCLK` (raise to 200000); see `FLASH-TIMING` note |
| Motor spins the wrong direction | `PH-DIR` — negate `duty_signed` or swap EN/PH pins in `src/pwm.cpp` |
| Current amplitude far above `TARGET_AMPS` | `ISENSE-SIGN` or `VREF` miscalibrated from Phase 1 |

---

### Phase 3 — Closed-Loop FOC, One Long Profile (`phase3`)

**Purpose.** Close the current loop and hold the board's top operating point for minutes. The
sketch builds ONE profile — a very long distance (25000 rad, ~4000 rotations) cruising at
1700 RPM after a ~2 s ramp — and hands it to both motors with `motorSetProfile()`. That is the
whole sketch: no ramp state machine and no elapsed-time accumulator on core0, because the control
ISR executes the profile itself.

At 1700 RPM the electrical frequency is:

```
f_electrical = 1700 RPM * 50 / 60 = 1417 Hz
```

which is the most demanding steady operating point the board sees. Current is fixed at
`MOTOR_AMPS` (0.8 A) and the voltage-limited derate is what keeps it there — at speed the drive
runs out of `ud = -we*Lq*iq` long before `uq`, so the derate trades current for speed rather than
letting the axis fall off the voltage limit.

The move takes about 2.4 minutes, which is the point: it is long enough to expose thermal drift,
integrator wind-up and any accumulating timing error.

**Flash command.**

```
cd bringup && pio run -e phase3 -t upload && pio device monitor
```

**What to watch.**

A small periodic print: profile time, each axis's commanded RPM, electrical degrees per control
step, bus voltage, and per-axis d/q currents and voltages. Watch for:

- `id` regulated near zero; `iq` near the 0.8 A command and falling once the `UD_FRAC` derate
  engages; `|ud|` riding just under its share of the bus once the axis is at speed.
- `degEStep` staying well under 90 (a quarter electrical cycle) — above roughly 20 degrees per
  step the current cannot look like a sine no matter how much voltage headroom there is.
- Phase currents are not in this print; scope them, or read `motorCurrentA/B` from
  `rotev_internal.h`, to confirm they stay smooth sinusoids through the ramp.
- Bus voltage sag under load — the derate sizes its limit against this number, so a dip costs real
  headroom.
- `motorProgress(m).done` staying false for the whole run, then the axes holding position.

On GPIO10, confirm the timing budget is still met at 1417 Hz electrical.

**Pass criteria.**

- Both motors reach and hold ~1700 RPM without stalling, hunting or audible pull-out.
- `id` stays near 0; no sustained oscillation or limit-cycling on either axis.
- The run completes without an overcurrent fault, nFAULT assertion or thermal shutdown.
- GPIO10 pulse width still within budget.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| Sustained oscillation or limit-cycling | `KP` / `KI` — lower `BANDWIDTH` and let both recompute |
| Current tracks sluggishly, large steady-state error | `KP` / `KI` — raise `BANDWIDTH` |
| Motor pulls out (loses sync) partway up the ramp | Lengthen the profile's accel time — `Profile::scaleTime()` on the phase's profile, or a lower `max_accel`; check bus sag |
| Noisy current signals causing instability | `OVERSAMPLE` — raise (watch timing budget); also check for ground loops on SOA/SOB |
| ADC sampling hits a switching edge (spikes on current traces) | `SAMPLE-INSTANT` — shift the ADC burst offset in `src/foc.cpp` `controlStep` toward the PWM center |
| Motor stalls under load | `MOTOR_AMPS` — the fixed 0.8 A command; verify the hardware limit before raising |

---

### Phase 4 — Back-and-Forth Profiles (`phase4`)

**Purpose.** Exercise the profile API the way an application does: start, finish, reverse, repeat.
The forward leg is built once with `Profile::fromVelAccel()`; the return leg is
`fwd.scaleDistance(-1.0f)`, i.e. the same envelope mirrored. Each time `motorProgress(m).done`
goes true the sketch issues the other leg. That is the entire loop.

This is the harshest test of the drive because every cycle contains a full accel, a full decel and
a direction reversal through zero speed, where the commanded field and the rotor are least
constrained by each other.

**Flash command.**

```
cd bringup && pio run -e phase4 -t upload && pio device monitor
```

**What to watch.**

The periodic print shows the cycle count plus the same per-axis telemetry as phase 3. Over each
leg the current frequency should rise through the accel ramp, hold flat through cruise, and fall
back through decel; the reversal should be clean rather than a lurch. The motor must return to
where it started every second leg — mark the shaft and check it after several cycles, since a
position error that repeats per cycle accumulates visibly.

On GPIO10, the timing budget is tightest at peak speed — confirm the pulse width stays in budget
throughout, not just at cruise.

**Pass criteria.**

- The axes complete the configured number of forward/back cycles and stop on target, with the
  shaft mark back where it started (within a mechanical degree or so).
- Peak velocity reaches the profile's `maxVelocity()`; `motorProgress()` reports `done` at the end
  of each leg.
- No overcurrent fault, driver nFAULT assertion, or thermal shutdown.
- GPIO10 pulse width within budget throughout the full profile.

**Failure modes and knobs.**

| Symptom | Check / Knob |
|---------|-------------|
| D-axis current grows significantly at high speed | Check `PHASE_LQ` in `src/constants.h` and `DECOUPLE_FRAC` (set 0 to A/B test the decoupling) |
| Shaft drifts from its start mark cycle after cycle | The axis is losing steps at the reversal or at peak speed, not accumulating profile error — the profile is exact. Lower peak velocity (`scaleTime()`) or check bus sag |
| Timing overrun at peak speed | `OVERSAMPLE` (drop to 2) or `SYSCLK` (raise the `set_sys_clock_khz` arg, e.g. 200000 = 200 MHz); see `FLASH-TIMING` note |
| Oscillation or instability at high speed | `KP` / `KI` — the plant changes at high electrical frequency; lower gains slightly |
| Hard stop / position error | `MOTOR_AMPS` not delivering enough torque, or the `UD_FRAC` derate cutting it back at speed; verify bus voltage and winding resistance |

---

### Phase 5 — Motor Characterization (`phase5`)

**Purpose.** Measure this motor's `PHASE_R`, `PHASE_LD` and `PHASE_LQ` rather than trusting the
datasheet, which is wrong on all three. Run it once per motor when a new motor type is fitted, and
put the results in `src/constants.h`; the measurement method and the values already there are
documented in that file's header comment.

This phase is the reason `rotev_internal.h` exists. It drives the inverter directly with
`motorSetVoltageAB()` for the DC and step-response measurements, uses `motorSetVelocity()` to
cruise at a fixed speed for the Lq sweep (and to re-align at theta 0 between stages), and reads
`motorCurrentD/Q()` and `motorVoltageD/Q()` for the regressions.

**Flash command.**

```
cd bringup && pio run -e phase5 -t upload && pio device monitor
```

**What to watch.** The sketch prints each stage's raw points and its regression result with an R².
Expect R² 0.998+ on the Ld step response and 0.999+ on the Lq sweep; a poor fit means the stage
did not hold its operating point, not that the motor is unusual.

The Lq sweep runs at `MOTOR_AMPS` (0.8 A), the same fixed current every closed-loop move uses. Its
speed points are chosen against the pull-out ceiling at that current — roughly `rpm * iq ~ 280` on
a 12 V bus — so raising them without lowering the current will pull the rotor out mid-sweep and
poison the fit.

---

## 3. Tunable Knobs Reference

These are the **only** values an operator should edit during bringup. Everything else in the
library is structural code that should not be changed without a design review.

| ID | Knob | File / Location | Default | When to Change |
|----|------|----------------|---------|---------------|
| `SYSCLK` | `set_sys_clock_khz(...)` in `hwInit()` | `src/hw.cpp` | `200000` (200 MHz) | Timing overrun at high speed — raise; re-verify clock stability and flash/SRAM access |
| `OVERSAMPLE` | `ADC_OVERSAMPLE` | `src/constants.h` | `2` | GPIO10 pulse too wide — drop to `1`; current waveform noisy — raise (watch timing budget) |
| `SAMPLE-INSTANT` | ADC burst placement in `controlStep` | `src/foc.cpp` `controlStep()` | Wrap-aligned (start of ISR) | Current spikes on scope indicate sampling lands on switching edge — shift the burst call toward PWM center |
| `KP` / `KI` | `KP_D`, `KP_Q`, `KI` (derived from `BANDWIDTH * PHASE_LD/LQ/R`) | `src/constants.h` | `3.79` / `8.28` / `4042` | Oscillation → lower; sluggish tracking → raise; adjust via `BANDWIDTH` and let the `constexpr` derivation recompute both axes |
| `BANDWIDTH` | `BANDWIDTH` | `src/constants.h` | `1000` (rad/s) | Primary knob for PI response speed; change this and `KP_D`/`KP_Q`/`KI` follow automatically |
| `MOTOR_AMPS` | `MOTOR_AMPS` | `src/constants.h` | `0.8` A | Fixed q-axis current for every closed-loop move, and not exposed to the API. Lower it to buy top speed (`rpm * iq ~ 280` on a 12 V bus), raise it for torque at the cost of that ceiling |
| `DECOUPLE_FRAC` | `DECOUPLE_FRAC` | `src/constants.h` | `1.0` | Cross-coupling feedforward strength; set `0.0` to A/B test whether it helps this machine |
| `IMAX_A` | `IMAX_A` | `src/constants.h` | `1.1` A | Current sense full-scale range; changes only if the shunt or amplifier gain does. Not a command clamp — the command is the fixed `MOTOR_AMPS` |
| `ADC_VREF` | `ADC_VREF` | `src/constants.h` | `3.3` V | Measured ADC reference differs from 3.3 V (use a multimeter on the AVDD pin) |
| `VBUS_V` | `VBUS_V` | `src/constants.h` | `12.0` V | Startup fallback only — the live bus comes from the ADS1015. Update if the supply is not 12 V |
| `ISENSE-SIGN` | Swap `ADC_SOA_*`/`ADC_SOB_*` channels or negate | `src/adc.cpp` `adcSampleMotor()` | As wired (SOA = phase A, SOB = phase B) | Current polarity inverted vs. expectation from Phase 1 |
| `PH-DIR` | Sign of `duty_signed` in `pwmSetPhase` | `src/pwm.cpp` `pwmSetPhase()` | Positive duty → PH high | Motor spins opposite to commanded direction |
| `LED-POL` | `ledDuty` active-low inversion | `src/foc_math.cpp` `ledDuty()` | Active-low (inverted) | Board turns out to have common-anode LED or opposite polarity convention |
| `CMD-DT` | `delayMicroseconds(500)` in the open-loop phase sketches | `bringup/phase1b_motor.h`, `phase2_openloop.h` | 500 µs | Rate at which core0 updates the open-loop commanded angle — reduce for a smoother field. Does not apply to phases 3 and 4: there the control ISR generates the angle from the profile, so core0 timing cannot reach it |
| `FLASH-TIMING` | Mark ISR chain `__not_in_flash_func` | `src/foc.cpp` `controlStep` and hot callees in `src/foc_math.cpp`, `src/pwm.cpp`, `src/adc.cpp`; `Profile::at` is `always_inline` in `src/profile.h` so it lands in `controlStep`'s own RAM section | Applied | GPIO10 pulse width is wide or jittery even after OVERSAMPLE/SYSCLK tuning — this indicates XIP flash-cache misses adding latency. Apply `__not_in_flash_func` to `controlStep` and the functions it calls (`adcSampleMotor`, `pwmSetPhase`, `piStep`, `park`, `inversePark`). This is a structural change, not a constant, so verify by re-scoping GPIO10 after applying it. |

### Notes on the Current Sense Range

The INA181A2 has a gain of 50 and the shunt is 30 mΩ, giving 1.5 V/A sensitivity. The ADC
reference is 3.3 V and the INA output is biased at 1.65 V (ISENSE_REF_V), so the usable range is
approximately ±1.1 A. The internal current command is clamped to that, well above the 0.8 A
`MOTOR_AMPS` every closed-loop move actually uses. If you need a higher current range you must
change the shunt resistor and update `SHUNT_OHMS`, `IMAX_A`, and `ISENSE_REF_V` in
`src/constants.h`.

### Note on Flash-Cache Jitter (FLASH-TIMING)

On the RP2350 with XIP flash, cache misses add variable latency (several hundred nanoseconds per
miss) to anything in the ISR chain. `pwmWrapISR` and `controlStep` are already marked
`__not_in_flash_func`, so they are placed in SRAM at link time. If the GPIO10 scope trace still
shows pulse widths that are wider than expected or vary cycle-to-cycle, extend the same marking to
the remaining callees (`adcSampleMotor`, `pwmSetPhase`, `piStep`, `park`, `inversePark`). This is a
structural change, not a constant, so re-scope GPIO10 after applying it to confirm the improvement.

---

## 4. Final Acceptance Checklist

Run through these checkboxes in order after all phases pass. Do not merge the feature branch
until all boxes are checked.

### Hardware pass criteria

- [ ] **Phase 1:** All four idle current traces within ±few mA of 0 A; `busVoltage()` within
      ±0.5V of multimeter reading; GO→green + audible tune, STOP→red; nSLEEP_1 and nSLEEP_2 both
      measure logic HIGH.
- [ ] **Phase 1b:** Two sinusoids ~90° apart at 5 RPM, amplitude ~`UQ_VOLTS / PHASE_R`, correct
      sign for the commanded direction.
- [ ] **Phase 1c:** `sensA` steady at `UQ_VOLTS / PHASE_R` with `sensB` near zero.
- [ ] **Phase 2:** Two sinusoidal currents at ~50 Hz electrical (60 RPM × 50/60), ~90° apart,
      amplitude near `TARGET_AMPS`; motor rotates smoothly at 60 RPM; GPIO10 pulse period
      ~41.6 µs, width within budget.
- [ ] **Phase 3:** Both axes hold ~1700 RPM (1417 Hz electrical) for the full ~2.4 minute profile;
      `id` near 0; no sustained oscillation; GPIO10 still within budget.
- [ ] **Phase 4:** Forward/back cycles complete and stop on target, shaft mark back where it
      started; no fault or thermal shutdown; GPIO10 within budget throughout.
- [ ] **Phase 5:** R, Ld and Lq measured with good fits, and `src/constants.h` updated if this is
      a new motor type.

### Build and test verification

- [ ] `pio test -e native` passes green (all unit tests pass).
- [ ] All bringup environments compile without warning:
  ```
  cd bringup && pio run
  ```
- [ ] Flash and RAM usage remain within comfortable margins (current builds: RAM ~3.5%, Flash ~0.5%
      of RP2350 totals — well clear of any limit).

### Documentation and wiring

- [ ] GPIO assignments in `src/constants.h` (nSLEEP, SOA/SOB, LED, buttons) match physical wiring.

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
  `// raised from 150000 to 200000: extra ISR timing margin for controlStep`).

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
cd bringup && pio run
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

*End of bringup guide. Once every phase passes and this checklist is complete, the library is
ready for integration.*
