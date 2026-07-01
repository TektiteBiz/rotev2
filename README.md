# RotEv2 FOC Library

PlatformIO/Arduino library for the **Tektite RotEv2** board (RP2354). Provides dual-core field-oriented control (FOC) for two independent 2-phase stepper motors, plus RGB LED and button I/O.

- Open-loop position control (theta_rad → Park/Clarke → PWM)
- Closed-loop current control (PI regulators, pole-placement tuned)
- Lag-compensation (cross-coupling decoupling, optional)
- Active-high buttons, active-low RGB LED

---

## Install (PlatformIO)

Add to `platformio.ini`:

```ini
[env:myboard]
platform     = https://github.com/maxgerhardt/platform-raspberrypi.git
board        = generic_rp2350
board_build.core = earlephilhower
framework    = arduino
lib_deps     = <path-or-registry-entry-for-rotev2>
build_flags  = -std=gnu++17
```

See `docs/BRINGUP.md` for hardware bring-up and first-boot validation steps.

---

## Dual-Core Contract

**The library owns core1.** The FOC loop runs on core1 and is started by `begin()`. User sketches run on core0 (`setup()` / `loop()`).

**Do NOT define `setup1()` or `loop1()` in your sketch** — doing so will conflict with the library's core1 entry point.

---

## Quick Start

```cpp
#include <rotev.h>
using namespace rotev;

void setup() {
  begin();
  motorEnable(MOTOR_1);
  ledColor(0, 0, 255);   // blue = running
}

void loop() {
  static float theta = 0;
  theta += 0.01f;
  motorWrite(theta, 0.3f, MOTOR_1);  // advance position, 0.3 A
  delay(1);
}
```

See `examples/Basic/Basic.ino` for the full example.

---

## Public API

All symbols are in `namespace rotev`. Include `<rotev.h>`.

### Initialization

```cpp
void begin();
```

Initializes GPIO, PWM, ADC, LED, and starts the FOC loop on core1. Call once from `setup()`.

### Motor Control

```cpp
void motorEnable(Motor m);
void motorDisable(Motor m);
void motorWrite(float theta_rad, float amps, Motor m);
```

- `motorEnable(m)` — releases nSLEEP and re-applies the last commanded setpoint. **Caveat:** the stored setpoint is the last value passed to `motorWrite()`, which defaults to 0 A at startup. To guarantee starting from rest, call `motorWrite(theta, 0.0f, m)` before `motorEnable(m)`.
- `motorDisable(m)` — zeros the current command and asserts nSLEEP.
- `motorWrite(theta_rad, amps, m)` — sets position angle (radians, any range) and d-axis current (q-axis target in amps). `amps` is clamped to ±1.1 A (sensor range).

### Lag Compensation

```cpp
void setLagComp(bool on);
```

Enables or disables cross-coupling (lag-compensation) decoupling in the FOC loop:
- Uq += ωe · Ld · Id
- Ud -= ωe · Lq · Iq

Defaults to disabled at startup.

### LED

```cpp
void ledColor(uint8_t r, uint8_t g, uint8_t b);
```

Sets the RGB LED. The LED is **active-low**, driven by PWM. Values are 0–255 (0 = full brightness, 255 = off).

### Buttons

```cpp
bool buttonPressed(Button b);
```

Returns `true` if the specified button is currently pressed. Buttons are active-high with internal pull-downs.

### Current Telemetry

```cpp
float motorCurrentA(Motor m);
float motorCurrentB(Motor m);
```

Returns the most recent phase A or phase B current reading (amps) from the FOC loop's ADC snapshot.

### Enums

```cpp
enum Motor  : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };
```

---

## GPIO / Bus Map

| Signal | GPIO | Notes |
|---|---|---|
| ENA_2 | 0 | Motor 2 phase A enable (DRV8825) |
| PHA_2 | 1 | Motor 2 phase A direction |
| ENB_2 | 2 | Motor 2 phase B enable |
| PHB_2 | 3 | Motor 2 phase B direction |
| ENA_1 | 4 | Motor 1 phase A enable |
| PHA_1 | 5 | Motor 1 phase A direction |
| ENB_1 | 6 | Motor 1 phase B enable |
| PHB_1 | 7 | Motor 1 phase B direction |
| LED_R | 8 | Red, active-low PWM |
| LED_G | 9 | Green, active-low PWM |
| SPI1_SCK / LOOP_TIMING | 10 | SPI1 SCK; also bringup loop-timing pin |
| SPI1_MOSI | 11 | SPI1 MOSI (user bus) |
| SPI1_MISO | 12 | SPI1 MISO (user bus) |
| SPI1_CS | 13 | SPI1 CS (user bus) |
| LED_B | 14 | Blue, active-low PWM |
| I2C0_SDA | 16 | Board has pull-ups fitted |
| I2C0_SCL | 17 | Board has pull-ups fitted |
| BTN_STOP | 19 | Active-high, internal pull-down |
| BTN_GO | 20 | Active-high, internal pull-down |
| nSLEEP_1 | 21 | Motor 1 driver sleep (active-low) |
| nSLEEP_2 | 22 | Motor 2 driver sleep (active-low) |
| SOB_1 / ADC0 | 26 | Motor 1 phase B current sense |
| SOA_1 / ADC1 | 27 | Motor 1 phase A current sense |
| SOB_2 / ADC2 | 28 | Motor 2 phase B current sense |
| SOA_2 / ADC3 | 29 | Motor 2 phase A current sense |

SPI1 (GPIO 10–13) and I2C0 (GPIO 16–17) are routed to headers and are usable as plain GPIO when not needed as buses.

---

## Motor Specifications (14HS11-1004)

| Parameter | Value |
|---|---|
| Step angle | 1.8° (200 steps/rev) |
| Pole pairs | 50 |
| Phase resistance | 3.5 Ω |
| Phase inductance | 3.5 mH (Ld = Lq) |
| Rated current | 1.0 A |

---

## FOC / PI Tuning

Pole-placement design: current loop bandwidth = 1000 rad/s.

```
kP = BANDWIDTH * L = 1000 * 0.0035 = 3.5
kI = BANDWIDTH * R = 1000 * 3.5    = 3500
```

Lag-compensation (when enabled via `setLagComp(true)`):
```
Uq += ωe * Ld * Id
Ud -= ωe * Lq * Iq
```

Bus voltage is assumed to be **12 V** (used behind the inverse-Park transform to normalize duty cycle). This will be replaced by a live ADC read in a future revision.

---

## Current Sensing

| Parameter | Value |
|---|---|
| Amplifier | INA186A3 |
| Shunt | 15 mΩ |
| Gain | 100 V/V |
| Reference | 1.65 V (mid-rail) |
| ADC reference | 3.3 V, 12-bit (0–4095) |
| Sensitivity | ≈ 0.54 mA/count |
| Range | ±1.1 A |

Commands passed to `motorWrite()` are clamped to ±1.1 A.

---

## Hardware Bring-Up

See [`docs/BRINGUP.md`](docs/BRINGUP.md) for step-by-step hardware validation, phase-by-phase test procedures, and expected oscilloscope waveforms.
