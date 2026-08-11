# RotEv2 FOC Library

PlatformIO/Arduino library for the **Tektite RotEv2** board (RP2354). Provides dual-core field-oriented control (FOC) for two independent 2-phase stepper motors, plus RGB LED and button I/O.

- Open-loop position control (theta_rad → inverse Park → PWM; no Clarke — phase currents are already in the αβ frame)
- Closed-loop current control (PI regulators, pole-placement tuned)
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
lib_deps     = https://github.com/Nv7-Github/rotev2.git
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
- `motorWrite(theta_rad, amps, m)` — sets the position angle (radians, any range) and the q-axis (torque) current command in amps. The d-axis current setpoint is held at 0. `amps` is clamped to ±1.1 A (sensor range).

### LED

```cpp
void ledColor(uint8_t r, uint8_t g, uint8_t b);
```

Sets the RGB LED. The LED is **active-low**, driven by PWM. Values are 0–255 (255 = full brightness, 0 = off).

### Buzzer

```cpp
void buzzerOn(uint16_t freq_hz);
void buzzerOff();
```

Drives the passive piezo buzzer on GPIO4 at 50% duty cycle. `freq_hz` is clamped to 1000-4000 Hz.
Calling `buzzerOn()` again while already sounding retunes the frequency without needing to call
`buzzerOff()` first.

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

### Enums

```cpp
enum Motor  : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };
enum AdcChannel : uint8_t { ADC_AIN1 = 0, ADC_AIN2 = 1, ADC_AIN3 = 2 };
```

---

## GPIO / Bus Map

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

SPI1 (GPIO 10–13) and I2C0 (GPIO 16–17) are routed to headers and are usable as plain GPIO when not needed as buses. GPIO 24/25 are general GPIO but cannot do PWM (they overlap the LED PWM slice).

---

## Motor Specifications (14HS11-1004)

| Parameter | Value |
|---|---|
| Step angle | 1.8° (200 steps/rev) |
| Pole pairs | 50 |
| Phase resistance | 4.042 Ω (measured, mean of 2 motors; datasheet says 3.5) |
| Phase inductance | Ld 3.79 mH, Lq 8.28 mH (measured, mean of 2; datasheet says 3.5) |
| Rated current | 1.0 A |

---

## FOC / PI Tuning

Pole-placement design: current loop bandwidth = 1000 rad/s.

```
kP_d = BANDWIDTH * Ld = 1000 * 0.0037946 = 3.79
kP_q = BANDWIDTH * Lq = 1000 * 0.0082764 = 8.28
kI   = BANDWIDTH * R  = 1000 * 4.0417   = 4042   (shared)
```

Bus voltage is read live from the ADS1015 (`busVoltage()`, ~1kHz) and used behind the inverse-Park
transform to normalize duty cycle. A nominal 12V fallback is used only until the first real ADC
sample lands at boot.

---

## Current Sensing

| Parameter | Value |
|---|---|
| Amplifier | INA181A2 |
| Shunt | 30 mΩ |
| Gain | 50 V/V |
| Reference | 1.65 V (mid-rail) |
| ADC reference | 3.3 V, 12-bit (0–4095) |
| Sensitivity | ≈ 0.54 mA/count |
| Range | ±1.1 A |

Commands passed to `motorWrite()` are clamped to ±1.1 A.

---

## Hardware Bring-Up

See [`docs/BRINGUP.md`](docs/BRINGUP.md) for step-by-step hardware validation, phase-by-phase test procedures, and expected oscilloscope waveforms.
