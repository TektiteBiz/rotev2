# RotEv2 FOC Library — Design Spec

**Date:** 2026-06-30
**Status:** Approved (brainstorming)
**Source PRD:** `docs/PRD.md`

## Overview

RotEv2 is a PlatformIO/Arduino library for the Tektite RotEv2 PCB (RP2354 MCU). It
implements heavily optimized open-loop-position / closed-loop-current stepper FOC
for two 2-phase stepper motors, plus RGB LED and button support. It is published as
a conventional Arduino/PlatformIO library while internally using Pico SDK headers for
center-aligned PWM, timed ADC sampling, and dual-core scheduling.

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Framework | earlephilhower arduino-pico core (import compatibility) with Pico SDK headers used internally | Users import as a normal Arduino/PlatformIO lib; internals get `<hardware/*.h>` for center-aligned PWM, ADC, DMA, IRQ. Verified RP2350/RP2354 (Pico 2) support. |
| Numerics | Single-precision `float` (M33 FPU) with standard `sinf`/`cosf` | Fast + readable; ~6200 cycles/loop @150 MHz vs a few hundred for trig. LUT only if bringup shows it doesn't fit. |
| Clock | Max validated clock; start 150 MHz, overclock target ~200 MHz confirmed in bringup | Timing constants derive from `clock_get_hz(clk_sys)`, never hard-coded. |
| Concurrency | FOC on **core1**, user code on **core0** | Real-time loop isolated from user code. Documented in README. |
| API style | Free functions in `namespace rotev` | Matches PRD examples; one fixed board. |
| ωe source | Differentiate commanded electrical angle + light LPF | Keeps `motorWrite(theta, current)` API exactly as PRD specifies. |
| Current sensing | INA186A3 (**100 V/V**), 15 mΩ shunt, REF 1.65 V → `V=1.65+1.5·I`, range ≈ ±1.1 A | Verified A3 = 100 V/V from TI datasheet. Current command clamped to ±1.1 A. |
| ADC oversampling | `ADC_OVERSAMPLE` default 4× per channel, burst centered on PWM midpoint | Rejects noise / boosts resolution while staying inside the center-aligned window (~16 µs burst). |
| Modulation | Direct αβ phase-voltage application via PH/EN (NOT SVPWM — that's 3-phase BLDC) | 2-phase stepper with two H-bridges; inverse-park voltages applied per phase. |
| LED polarity | Active-low (PWM on cathodes): pin high = off, pin low = on | Confirmed against board. |

## Hardware Reference

MCU: RP2354. GPIO map (from PRD):

| GPIO | Label | | GPIO | Label |
|------|-------|-|------|-------|
| 0 | ENA_2 | | 14 | LED.B |
| 1 | PHA_2 | | 19 | BTN_STOP |
| 2 | ENB_2 | | 20 | BTN_GO |
| 3 | PHB_2 | | 21 | nSLEEP_1 |
| 4 | ENA_1 | | 22 | nSLEEP_2 |
| 5 | PHA_1 | | 26 | SOB_1 (ADC0) |
| 6 | ENB_1 | | 27 | SOA_1 (ADC1) |
| 7 | PHB_1 | | 28 | SOB_2 (ADC2) |
| 8 | LED.R | | 29 | SOA_2 (ADC3) |
| 9 | LED.G | | | |

**User/bringup buses:**
- SPI1: GPIO10 SCK, 11 MOSI, 12 MISO, 13 CS — user-facing in production; **available for bringup debug**. `LOOP_TIMING_PIN` defaults to **GPIO10** during bringup.
- I2C0: GPIO16 SDA, 17 SCL — user-facing (board has pull-ups); not used in bringup.
- All bus pins may also serve as plain GPIO.

**Drivers:** Two motors, each with phase A and B. Each phase = one DRV8874 in PH/EN
mode + one INA186A3 inline current sensor (15 mΩ shunt, REF 1.65 V). `nSLEEP_1`
gates both motor-1 drivers; `nSLEEP_2` both motor-2 drivers.

**Motor (14HS11-1004):** 1.8°/step (200 steps/rev → **50 pole pairs**), phase R = 3.5 Ω,
L = 3.5 mH, rated 1.0 A.

**Buttons:** active-high, require internal pull-downs.

**RGB LED:** PWM on GPIO 8/9/14. **Active-low** — the PWM pins drive the LED
cathodes, so pin high = off, pin low = on. `LED_ACTIVE_HIGH = false`; a brightness
value of 255 = full on = pin held low (duty inverted internally).

## Architecture

### Packaging / layout
```
rotev2/
  library.json, library.properties, keywords.txt
  src/            rotev.h + internal modules
  examples/       short published usage sketches
  bringup/        self-contained PlatformIO project (lib_deps -> ../, own platformio.ini)
    phase1_hw/  phase2_openloop/  phase3_foc/  phase4_full/
  docs/PRD.md, docs/BRINGUP.md, docs/superpowers/specs/...
  README.md
```

### Modules (`src/`), each one focused unit
- `rotev.h` / `rotev.cpp` — public API (`namespace rotev`); thin, delegates to internals.
- `foc.*` — control loop: park / inverse-park, PI, lag comp. Runs on core1.
- `pwm.*` — center-aligned 24 kHz PWM setup; per-phase duty + direction (PH/EN).
- `adc.*` — ADC config, timed oversampled sampling, counts→amps.
- `hw.*` — pin map, `nSLEEP`, clock/overclock setup.
- `led.*` — RGB PWM.
- `button.*` — pull-down config + debounced reads.
- `debug.*` — `LOOP_TIMING_PIN` toggling (compile-time `ENABLE_LOOP_TIMING`).

### Dual-core model & data flow
- **core0:** user code + `rotev::` API.
- **core1:** real-time FOC; serviced by the 24 kHz PWM-wrap IRQ. Even/odd IRQs
  alternate motor 1 / motor 2 → **12 kHz per motor**.
- **Cross-core handoff:** `motorWrite` publishes `{theta, iq_cmd, enabled}` per motor
  into a shared struct; publication made atomic via an RP2350 hardware spinlock (or
  seqlock snapshot) so the ISR reads a consistent set. Telemetry (measured currents,
  estimated ωe) returns the same way for core0 logging/plotting.

## Control Loop

### PWM
- 4 phase EN pins → PWM slices, phase-correct (center-aligned), 24 kHz.
- `TOP = f_sys / (2 · 24000 · div)` (e.g. 3125 @150 MHz) → ~11–12-bit duty.
- One slice wrap IRQ = master 24 kHz tick on core1.

### Modulation (2-phase, direct αβ — NOT SVPWM)
SVPWM is a 3-phase BLDC technique and does not apply here. This is a 2-phase stepper
with two independent H-bridges: inverse-park yields the two phase voltages `Vα`
(phase A), `Vβ` (phase B) directly, and each is applied as a sinusoidal bipolar phase
voltage via `PH = sign(V)`, `EN duty = |V|/Vbus`. Open-loop phase current traces two
90°-shifted sine waves.

### ADC sampling
- Single SAR ADC, round-robin over the **active motor's two channels** only.
- Each channel sampled `ADC_OVERSAMPLE` (default 4×) in a tight burst, integer-
  accumulated then divided, then converted. Burst centered on the PWM midpoint (start
  ~half-a-burst before wrap so samples straddle center). ~16 µs @4×, inside budget.
  Drop to 2× if bringup shows the window too wide.
- `I = (counts/4095 · Vref − 1.65) / 1.5`; `Vref` isolated in one constant.

### Per-motor step (12 kHz, in ISR)
1. Read `Iα, Iβ` (measured; already αβ, no Clarke).
2. `θe = 50·θ_cmd`; estimate `ωe = 50·Δθ_cmd/Δt` with light LPF.
3. **Park** using commanded `θe` (open-loop position assumption) → `Id, Iq`.
4. **PI**: `kP = BW·L = 3.5`, `kI = BW·R = 3500` (BW = 1000 rad/s), anti-windup clamp.
   Setpoints `Id* = 0`, `Iq* = current command`.
5. **Lag comp**: `Uq = PIq(Iq*−Iq) + ωe·Ld·Id`, `Ud = PId(0−Id) − ωe·Lq·Iq`
   (Ld = Lq = 3.5 mH).
6. **Inverse Park** → `Vα, Vβ` → duty/PH. The 12 V bus assumption and all
   decoupling/feedforward live behind one `inversePark(Ud, Uq, θe, Vbus)` boundary so
   swapping in an ADC bus-voltage reading later is a one-line change.

Disabled motor: PI state reset, outputs zeroed, `nSLEEP` low.

### Debug timing pin
`LOOP_TIMING_PIN` (default GPIO10, bringup only) driven high on ISR entry, low on exit.
Scope it against PWM/current to measure per-loop compute time and sampling position.
`ENABLE_LOOP_TIMING` off in production frees the SPI1 pin.

## Public API (`namespace rotev`)
```cpp
void begin();                                   // clocks, pins, PWM, ADC, launch core1 FOC
void motorEnable(Motor m);                      // MOTOR_1 / MOTOR_2
void motorDisable(Motor m);
void motorWrite(float theta_rad, float amps, Motor m);   // position + q-current, amps clamped ±1.1A
void ledColor(uint8_t r, uint8_t g, uint8_t b);
bool buttonPressed(Button b);                   // BTN_STOP / BTN_GO
float motorCurrentA(Motor m), motorCurrentB(Motor m);    // last measured phase amps (telemetry)
```
`Motor` and `Button` are enums.

## Bringup (see `docs/BRINGUP.md` for full procedure)

Each phase is a sketch under `bringup/` with documented pass/fail vs scope/plotter.

- **Phase 1 — Basic HW:** enable/disable drivers, stream both phase currents, LED
  color on STOP/GO. *Pass:* sensible currents, buttons+LED respond, `nSLEEP` toggles.
- **Phase 2 — Open-loop SVPWM:** open-loop 60 RPM, voltage set for 0.1 A phase
  current; plot currents. *Pass:* two clean ~90°-shifted sine waves; timing pin shows
  stable 24 kHz.
- **Phase 3 — Closed-loop FOC (no lag comp):** PI current control, constant velocity /
  S-curve. *Pass:* Iq tracks setpoint, Id≈0, stable spin.
- **Phase 4 — Full library:** enable lag comp; S-curves to 100 rotations and 300 RPM.
  *Pass:* stable to 300 RPM, Id≈0 under speed (decoupling working), loop within budget.

## Documentation Deliverables
- `README.md` — library usage; full GPIO/bus map (incl. SPI1 GPIO10–13 & I2C0
  GPIO16–17); motor specs; PI tuning; current-sense range; dual-core note.
- `docs/BRINGUP.md` — phased procedure, what to look for, pass/fail.

## Testing
Real-time FOC validated on-hardware via bringup phases (no host harness for the ISR
path). Pure math helpers (park/inverse-park, scaling, ωe estimate) factored to be
host-testable for optional lightweight native tests.

## Open Assumptions to Confirm on Hardware
- Overclock target (~200 MHz) stability.

**Resolved:** ADC `Vref` = 3.3 V (confirmed); LED active-low with PWM on cathodes
(confirmed); modulation is direct αβ phase-voltage application, not SVPWM (confirmed).

## Workflow (per CLAUDE.md)
Feature branch, pre-feature tag, commit spec, per-phase validation against ground
truth before advancing; no direct work on `main`.
