# RotEv2 FOC Library

## Install (PlatformIO)

Add to `platformio.ini`:

```ini
[env:myboard]
platform     = https://github.com/maxgerhardt/platform-raspberrypi.git
board        = generic_rp2350
board_build.core = earlephilhower
framework    = arduino
lib_deps     = https://github.com/TekiteBiz/rotev2.git
build_flags  = -std=gnu++17
```

---

## Dual-Core Contract

**The library owns core1.** The 24 kHz control ISR runs there and is started by `begin()`. Your
sketch runs on core0 (`setup()` / `loop()`).

**Do NOT define `setup1()` or `loop1()` in your sketch** — that conflicts with the library's core1
entry point.

---

## Quick Start

```cpp
#include <rotev.h>
using namespace rotev;

void setup() {
  Serial.begin(115200);
  begin();

  // 100 rad forward, cruising at 100 rad/s, reaching that in 0.5 s.
  Profile p = Profile::fromVelAccel(100.0f, 100.0f, 200.0f);
  p.print();

  motorEnable(MOTOR_1);
  motorSetProfile(MOTOR_1, p);
  ledColor(0, 0, 255);  // blue = running
}

void loop() {
  ProfileState st = motorProgress(MOTOR_1);
  Serial.printf("t %.2f s  pos %.1f rad  vel %.1f rad/s\n", st.t, st.pos, st.vel);
  if (st.done) ledColor(0, 255, 0);  // green = arrived
  delay(100);
}
```

See `examples/Basic/Basic.ino`.

---

## Profile

A `Profile` is a complete motion plan: a distance, and the velocity/acceleration envelope used to
cover it.

The shape is trapezoidal in velocity — constant acceleration up to the cruise speed, constant
speed, then constant deceleration back to rest — so position traces the familiar S. A move too
short to reach its cruise speed before it must start stopping degenerates to a triangle
(`cruiseTime() == 0`) at the same acceleration.

Distances are **signed**: a negative distance runs the move backwards. Velocity and acceleration
arguments are magnitudes.

### Constructing

```cpp
Profile();                                                        // empty, valid() == false
static Profile fromVelAccel(float distance, float max_vel, float max_accel);
static Profile fromTimeAccel(float distance, float time, float max_accel);
Profile scaleDistance(float k) const;
Profile scaleTime(float k) const;
```

- **`fromVelAccel(distance, max_vel, max_accel)`** — the usual one. Cover `distance` rad, never
  exceeding `max_vel` rad/s or `max_accel` rad/s². The duration falls out of the numbers.

  ```cpp
  Profile p = Profile::fromVelAccel(100.0f, 100.0f, 200.0f);  // ~1.5 s
  ```

- **`fromTimeAccel(distance, time, max_accel)`** — cover `distance` rad in exactly `time` seconds
  without exceeding `max_accel`. The cruise velocity is solved for. If `time` is shorter than the
  fastest move `max_accel` allows, you get that fastest move instead, and `duration()` is then
  larger than the `time` you asked for — check it if that matters.

  ```cpp
  Profile p = Profile::fromTimeAccel(100.0f, 2.0f, 200.0f);   // exactly 2 s
  ```

- **`scaleDistance(k)`** — vertical stretch of an existing profile. Same durations; distance,
  velocity and acceleration all scale by `k`. A negative `k` reverses the move, which is how you
  build a return leg:

  ```cpp
  Profile back = p.scaleDistance(-1.0f);  // same envelope, opposite direction
  ```

- **`scaleTime(k)`** — horizontal stretch. Same distance; every duration scales by `k`, so
  velocity scales by `1/k` and acceleration by `1/k²`. `k` must be positive.

  ```cpp
  Profile gentle = p.scaleTime(2.0f);  // same move, twice as long, quarter the accel
  ```

Bad arguments (zero, negative or non-finite distance/velocity/accel/time) produce an **empty**
profile rather than a fault: `valid()` is `false`, everything reads zero, and handing it to a
motor simply commands no motion.

### Querying

```cpp
float distance()    const;  // signed, radians
float duration()    const;  // seconds, accel + cruise + decel
float maxVelocity() const;  // signed peak (cruise) velocity, rad/s
float maxAccel()    const;  // signed peak acceleration, rad/s^2
float accelTime()   const;  // seconds
float cruiseTime()  const;  // seconds, 0 for a triangular profile
float decelTime()   const;  // seconds, always == accelTime()
bool  valid()       const;  // false for an empty/degenerate profile
```

`maxVelocity()` and `maxAccel()` carry the sign of the move, so they are the *actual* peak values
reached, not the magnitudes you passed in — for a triangular profile `maxVelocity()` is below the
`max_vel` you asked for.

### Sampling

```cpp
ProfileState at(float t) const;   // t clamped to [0, duration()]

struct ProfileState {
  float t;    // seconds since the profile started
  float pos;  // radians travelled (signed)
  float vel;  // rad/s (signed)
  float acc;  // rad/s^2 (signed)
  bool  done; // t >= duration()
};
```

`at()` evaluates the profile at any time without a motor being involved — useful for plotting or
for checking a move before committing to it. `t` is clamped, so `at(0)` is the start and any `t`
past the end returns the final position with zero velocity and `done == true`.

### Printing

```cpp
void print() const;
```

Dumps the profile to `Serial` in a readable block:

```
Profile: 100.000 rad in 1.500 s
  peak vel   100.000 rad/s (954.9 rpm)
  peak accel 200.000 rad/s^2
  accel      0.500 s   cruise 0.500 s   decel 0.500 s
```

An empty profile prints `Profile: <empty>`.

---

## Public API

All symbols are in `namespace rotev`. Include `<rotev.h>`.

### Initialization

```cpp
void begin();
```

Initializes GPIO, PWM, both ADCs, the LED and the buzzer, and starts the control loop on core1.
Call once from `setup()`.

### Motor

```cpp
void motorEnable(Motor m, bool enable = true);
void motorSetProfile(Motor m, const Profile& p);
Profile      motorProfile(Motor m);
ProfileState motorProgress(Motor m);
```

- **`motorEnable(m)`** — releases the driver's nSLEEP and lets the axis drive current.
  `motorEnable(m, false)` zeroes the current and puts the driver back to sleep.
- **`motorSetProfile(m, p)`** — starts `p` immediately, from wherever the axis currently is.
  Profiles are **relative**: the distance is measured from the position at the moment you call
  this, and the profile clock restarts at 0. Calling it again mid-move abandons the old profile
  and starts the new one from the current position. Setting a profile on a disabled motor stores
  it; it starts running when you enable the motor.
- **`motorProfile(m)`** — a copy of the profile currently executing.
- **`motorProgress(m)`** — where that profile is right now: elapsed time, position, velocity,
  acceleration, and whether it has finished. Poll `.done` to sequence moves.

When a profile finishes, the axis **holds**: velocity zero, angle frozen, still energised, so it
resists being pushed off target. Call `motorEnable(m, false)` to release it.

### LED

```cpp
void ledColor(uint8_t r, uint8_t g, uint8_t b);
```

Sets the RGB LED, 0–255 per channel (0 = off, 255 = full brightness).

### Buzzer

```cpp
void buzzerOn(uint16_t freq_hz);
void buzzerOff();
```

Drives the passive piezo buzzer at 50% duty. `freq_hz` is clamped to 1000–4000 Hz. Calling
`buzzerOn()` again while sounding retunes the frequency; you do not need `buzzerOff()` in between.

### Buttons

```cpp
bool buttonPressed(Button b);
```

`true` while the button is held.

### ADC and Bus Voltage

```cpp
float adcRead(AdcChannel ch);  // ADC_AIN1 / ADC_AIN2 / ADC_AIN3, volts
float busVoltage();            // volts
```

An on-board ADS1015 samples the motor bus voltage and the three user channels (AIN1–3) in the
background — there is nothing to start or poll. A timer started by `begin()` round-robins the four
channels, weighted so bus voltage refreshes at roughly 1 kHz (the control loop needs it) and each
user channel at roughly 330 Hz. Both calls return the most recent cached sample in volts and
are non-blocking.

### Enums

```cpp
enum Motor      : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button     : uint8_t { BTN_STOP = 0, BTN_GO = 1 };
enum AdcChannel : uint8_t { ADC_AIN1 = 0, ADC_AIN2 = 1, ADC_AIN3 = 2 };
```
