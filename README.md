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
static Profile fromAccelDecel(float distance, float max_vel, float max_accel, float max_decel);
static Profile fromTimeAccelDecel(float distance, float time, float max_accel, float max_decel);
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

- **`fromAccelDecel(distance, max_vel, max_accel, max_decel)`** — the asymmetric complement to
  `fromVelAccel`: the ramp up runs at `max_accel` and the ramp down at `max_decel`. Useful when the
  axis can brake harder than it can drive (or the reverse). Passing the same rate twice reproduces
  `fromVelAccel` exactly.

  ```cpp
  Profile p = Profile::fromAccelDecel(100.0f, 100.0f, 100.0f, 400.0f);  // gentle start, hard stop
  ```

- **`fromTimeAccelDecel(distance, time, max_accel, max_decel)`** — the asymmetric complement to
  `fromTimeAccel`, with the same "too short a time gives you the fastest move instead" fallback.

  ```cpp
  Profile p = Profile::fromTimeAccelDecel(100.0f, 2.0f, 100.0f, 400.0f);
  ```

- **`scaleDistance(k)`** — vertical stretch of an existing profile. Same durations; distance,
  velocity, acceleration and deceleration all scale by `k`, so an asymmetric profile keeps its
  accel:decel ratio. A negative `k` reverses the move, which is how you build a return leg:

  ```cpp
  Profile back = p.scaleDistance(-1.0f);  // same envelope, opposite direction
  ```

- **`scaleTime(k)`** — horizontal stretch. Same distance; every duration scales by `k`, so
  velocity scales by `1/k` and both acceleration and deceleration by `1/k²`. `k` must be positive.

  ```cpp
  Profile gentle = p.scaleTime(2.0f);  // same move, twice as long, quarter the accel
  ```

### Multi-leg moves

```cpp
struct Leg {
  float dist;   // signed, rad
  float accel;  // magnitude, rad/s^2
  float decel;  // magnitude, rad/s^2
};

static bool fromLegs(const Leg* legs, int n, float time, Profile* out);
```

`fromLegs()` solves a whole sequence at once: it finds the single cruise velocity at which the legs,
run back to back, take exactly `time` seconds, and writes the resulting profiles to `out[0..n)`.
Each leg keeps its own accel and decel. Sharing one cruise velocity is what makes the sequence read
as one continuous move rather than n unrelated ones.

```cpp
const Leg legs[] = {{3.0f, 1.75f, 4.78f}, {7.0f, 1.0f, 1.75f}, {0.5f, 1.75f, 4.78f}};
Profile out[3];
if (!Profile::fromLegs(legs, 3, 9.9f, out)) { /* budget too tight -- see below */ }
```

```
leg1   3.00 rad v=1.3451 ta 0.769 tc 1.705 td 0.281  T 2.7554
leg2   7.00 rad v=1.3451 ta 1.345 tc 4.147 td 0.769  T 6.2611
leg3   0.50 rad v=1.1318 ta 0.647 tc 0.000 td 0.237  T 0.8835   <- pinned triangular
```

A leg too short to reach the shared velocity — leg 3 above, whose ramps meet at 1.132 rad/s — simply
runs flat out as a triangle and takes what it takes; the solver slows the other legs to absorb the
difference so the total still lands on the budget. Getting this wrong is subtle: a saturated leg's
duration stops responding to the cruise velocity entirely, so a solver that keeps crediting it with
the trapezoid formula lands a few milliseconds short.

The return value is **false** when `time` is below the flat-out total (every leg triangular at its
own limits). `out[]` is still filled, with that flat-out move — runnable, just slower than you asked
for — so you can check `duration()` to see what was actually possible. Degenerate input (`n <= 0`,
a null array, or a leg with a zero/non-finite distance, accel or decel) leaves every `out[]` entry
empty and returns false. Nothing is allocated; both arrays are caller-owned.

Bad arguments (zero, negative or non-finite distance/velocity/accel/time) produce an **empty**
profile rather than a fault: `valid()` is `false`, everything reads zero, and handing it to a
motor simply commands no motion.

### Querying

```cpp
float distance()    const;  // signed, radians
float duration()    const;  // seconds, accel + cruise + decel
float maxVelocity() const;  // signed peak (cruise) velocity, rad/s
float maxAccel()    const;  // signed peak acceleration, rad/s^2
float maxDecel()    const;  // signed peak deceleration magnitude, rad/s^2
float accelTime()   const;  // seconds
float cruiseTime()  const;  // seconds, 0 for a triangular profile
float decelTime()   const;  // seconds, == accelTime() only for a symmetric profile
bool  symmetric()   const;  // true when the two ramps take the same time
bool  valid()       const;  // false for an empty/degenerate profile
```

`maxVelocity()`, `maxAccel()` and `maxDecel()` carry the sign of the move, so they are the *actual*
peak values reached, not the magnitudes you passed in — for a triangular profile `maxVelocity()` is
below the `max_vel` you asked for. `maxDecel()` is a magnitude in the sense that the acceleration
actually reported during the ramp down is its negative: a profile decelerating at 400 rad/s² reads
`maxDecel() == 400` and `at(t).acc == -400` there.

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
void printWithUnits(float wheel_radius, const char* unit) const;
```

`print()` dumps the profile to `Serial` in a readable block, in radians:

```
Profile: 100.000 rad in 1.500 s
  peak vel   100.000 rad/s (954.9 rpm) held 0.500 s
  peak accel 200.000 rad/s^2   peak decel 200.000 rad/s^2
  accel      0.500 s   cruise 0.500 s   decel 0.500 s
```

"held" is the time spent at peak velocity — the cruise time, `0.000` for a triangular profile.

`printWithUnits(wheel_radius, unit)` prints the same block converted to linear units, multiplying
every radian by `wheel_radius` and labelling it `unit`. The rpm figure is radius-independent and
stays. For a 16 mm wheel:

```cpp
p.printWithUnits(0.016f, "m");
```

```
Profile: 1.600 m in 1.500 s
  peak vel   1.600 m/s (954.9 rpm) held 0.500 s
  peak accel 3.200 m/s^2   peak decel 3.200 m/s^2
  accel      0.500 s   cruise 0.500 s   decel 0.500 s
```

An empty profile prints `Profile: <empty>` either way.

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

- **`motorEnable(m)`** — releases the driver's nSLEEP and lets the axis drive current. The axis
  then **holds position** even if it has never been given a profile: it is energised at standstill
  and resists being turned by hand, drawing the same `MOTOR_AMPS` a finished move holds at. There
  is no position sensor, so "position" means the commanded angle, which is 0 at boot — the first
  enable therefore pulls the rotor into alignment with electrical angle 0 rather than holding
  wherever it physically sits. `motorEnable(m, false)` zeroes the current and puts the driver back
  to sleep, leaving the shaft free.
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
