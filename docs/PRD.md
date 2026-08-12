# rotev2 PRD
RotEv2 is a PlatformIO library meant specifically for the Tektite RotEv2 PCB and implements heavily optimized open-loop stepper FOC for both motors.

# Hardware
The library has 3 main functions exposed to the user:
- Stepper FOC control (via the drivers and current sensors)
- RGB LED
- Button detection
Documentation must be written in README.md outlining each function and explaining its use as well as the user-exposed GPIOs.

## MCU
MCU: RP2354
GPIO mappings: (GPIO #: label)
- 0: PWMA_2
- 1: PWMB_2
- 2: PWMA_1
- 3: PWMB_1
- 4: BUZZ
- 8: LED.R
- 9: LED.G
- 14: LED.B
- 18: I2C1_SDA
- 19: I2C1_SCL
- 20: BTN_STOP
- 21: BTN_GO
- 22: nSLEEP_1
- 23: nSLEEP_2
- 26: SOB_1
- 27: SOA_1
- 28: SOB_2
- 29: SOA_2

## Drivers
There are two stepper motors (motor 1 and motor 2). Each motor has phases A and B. Each phase has a DRV8874 motor driver set up with PH/EN mode, and a current sensor which is a INA181A2 with a 30mohm shunt and with REF biased to 1.65V. SOB_1 is the current output of the B phase of motor 1.

Each driver's EN pin is hardwired HIGH (pull-up on the PCB, no MCU connection) rather than GPIO-controlled. PH is driven by PWM (locked-antiphase: duty=(1+d)/2, d in [-1,1]), so current is always actively driven through the H-bridge rather than coasting -- this is the decay behavior FOC needs. For example, PHB_2 is the PWM pin on the B phase driver for motor 2. This replaced an earlier sign-magnitude scheme (EN-PWM, PH as static direction) that coasted instead of actively decaying; see Known Pitfalls.

nSLEEP is connected between each pair, so both drivers for motor 1 share nSLEEP_1 and both drivers for motor 2 share nSLEEP_2

## Buttons
The buttons go HIGH when pressed, so need the internal pull-down. 

## Buzzer
There is a passive piezo buzzer with a range of 1k-4khz on the BUZZ pin, apply a 50% duty cycle and adjust frequency. Make this user facing so the user can turn on and off the buzzer and set the frequency.

## RGB LED
The RGB LED will be controlled via PWM with the 3 pins. 

## Additional GPIOs
GPIOs 10, 11, 12, and 13 are meant for use by the user, but will also be used during bringup (explained later). These pins are the SPI1 bus, which must be documented in the readme. GPIOs 16 and 17 are I2C0 and feature I2C pull-ups on the board and are also exposed (but not used for bringup) and also must be documented in README.md. These pins serve as busses for the user but can also be used as general GPIO and not exclusively for SPI/I2C. The mappings when being used as a bus are as follows:
- 6, 7: General GPIO (PWM-capable)
- 10: SCK
- 11: MOSI
- 12: MISO
- 13: CS
- 16: SDA
- 17: SCL
- 24, 25: General GPIO, cannot do PWM since overlaps with LED PWM slice

## ADS1015 ADC
On I2C1 (GPIO 18/19) there is an ADS1015IRUGR ADC. AIN0 on this ADC is connected to bus voltage with a voltage divider with 7.3kohm on the high side and 2.2kohm on the low side. This needs to be sampled at ~1khz for the FOC loop inverse park. AIN1-3 are exposed to the user so consider how to display them to the user too.

The driver samples in a repeating-timer callback doing blocking I2C. Measured
in service the bus reads 12.12-12.17 V with a 0.07 V spread, i.e. neither sag
nor meaningful sampling noise, so the live value behaves like the 12.0 V
constant it replaced. Two caveats worth keeping in mind: the I2C return codes
are currently ignored, so a failed transaction writes uninitialised stack into
the cached voltage; and a bus voltage of 0 (drivers unpowered) sends
inversePark down its `inv = 0` path and trips the voltage backstop, so the FOC
is silently inert with driver power off.

# FOC
High frequency FOC will be implemented and run. The specifications are:
- 24kHz PWM frequency (with 12kHz control loop for each motor, alternating between motor 1 and motor 2)
- Need to be able to disable the control and each driver via nSLEEP (e.g. motorEnable(MOTOR_1) type of thing)
- The caller does not command position, velocity or current directly. It hands
  the ISR a motion PROFILE and the ISR executes it (`motorSetProfile(m, p)`).
  The profile computation has to happen inside the FOC tight loop: a commanded
  field generated on core0 is only as fine as the caller's loop rate and
  freezes for the whole of any caller stall, which lands directly on the field
  as torque ripple. Generating it on the hardware-timed control tick removes
  the caller's timing from the picture entirely.
- `Profile` (see `src/profile.h`) is trapezoidal in velocity -- constant
  acceleration to a cruise speed, then constant deceleration -- so the position
  it integrates to is the S curve. It is constructible from distance plus max
  velocity and acceleration, from distance plus a target time and acceleration,
  or by scaling an existing profile's distance or time.
- Current is fixed internally at `MOTOR_AMPS` (0.8 A) and is not exposed.
- The caller can read back the active profile (`motorProfile`) and its progress
  -- time, position, velocity, acceleration, done (`motorProgress`).
- Telemetry needed for bringup and tuning: per-motor phase currents, dq
  currents and applied dq voltages, plus live bus voltage. Without dq
  telemetry the loop is effectively unobservable -- see Known Pitfalls. This
  is not part of the user-facing API; it lives in `src/rotev_internal.h`.

Implementation-wise, there must be
- Center aligned PWM for SVPWM
- No clarke transform is needed since the currents are already in alpha-beta frame
- For park transform, assume the rotor is at the target position, and run PI control
- Run PI with pole placement, PER AXIS (the machine is salient, so a single
  gain cannot place both poles):
kP_d = BANDWIDTH * Ld
kP_q = BANDWIDTH * Lq
kI   = BANDWIDTH * R      (SHARED between the two axes)

kI is deliberately not split. Each axis must place its controller zero on its
own plant pole: the d-axis plant is 1/(Ld*s + R) with a pole at -R/Ld, so the
zero belongs at kI/kP_d = R/Ld, which falls out of kP_d = BW*Ld with a shared
kI = BW*R. The same holds for q. The saliency lives entirely in kP.

The motor specs are follows (write this in README):
- Part No: 14HS11-1004
- Step Angle: 1.8deg
- Phase Resistance: 3.5ohms  (datasheet -- WRONG, see below)
- Inductance: 3.5mH          (datasheet -- WRONG, see below)

### Measured motor parameters (these supersede the datasheet)

Both datasheet figures are wrong, and the inductance is wrong by more than 2x
in the direction that costs top speed. Bringup phase 5 measures them on real
hardware. The values below are the mean of two motors, which agreed to within
1.6% on R, 1.5% on Ld and 3.6% on Lq.

| Parameter | Measured | Datasheet |
|---|---|---|
| R (winding + Rds(on) + shunt) | 4.0417 ohm | 3.5 |
| Ld | 3.7946 mH | 3.5 |
| Lq | 8.2764 mH | 3.5 |
| saliency Lq/Ld | 2.16 - 2.20 | not given |

**The machine is salient.** Lq is ~2.2x Ld, which is expected for a hybrid
stepper -- the magnet sits in the d-axis flux path with mu_r ~ 1, making d the
high-reluctance axis -- but it means one inductance constant cannot describe
both axes. The datasheet's 3.5 mH is roughly Ld; nobody published Lq, and Lq
is the one that governs high-speed behaviour.

**Lq falls with current** (iron saturation), roughly 8.3 mH at 0.5 A trending
toward 6 mH at 1.0 A. The runtime therefore does not trust the constant: it
maintains a per-motor ONLINE Lq estimate from |ud|/(we*|iq|). PHASE_LQ only
seeds that estimate and sets kP_q and the decoupling feedforward.

**lambda_m (magnet flux) is NOT reliably measured.** The phase 5 extraction
returns a negative value with poor fit quality on both motors, and the two
motors disagree by 10x on it while agreeing to within 4% on everything else --
so the method is broken, not the motors. Do not use the printed Kt. Two
independent facts bound it instead: the 1700 RPM ceiling requires
lambda_m <= 0.67 mWb, hence Kt <= 0.033 N.m/A. That is far below what the
datasheet holding torque suggests, and it explains why the drive runs out of
torque so readily at low current.

Use the resistance and inductances for the PI tuning above.
Use bandwidth = ~160 Hz (so 1000 rad/s)

### Cross-coupling decoupling ("lag compensation")

Ud_compensation = −ωe · Lq · Iq
Uq_compensation = +ωe · Ld · Id

So in the end:
Uq = PIq(Iq_setpoint − Iq_measured) + ωe·Ld·Id_setpoint
Ud = PId(0 − Id_measured) − ωe·Lq·Iq_setpoint

**Feed these from the current COMMAND, never the measurement.** Using measured
Id/Iq closes a loop Ud -> Id -> Uq -> Iq -> Ud whose gain is
(we*Lq)(we*Ld)/(Zd*Zq); at speed Zd -> we*Ld and Zq -> we*Lq, so the gain
approaches exactly 1 -- marginally stable before the pipeline delay is even
counted -- and it multiplies current-sense ripple by we*Lq, which is 30 V/A at
690 RPM. Measured on hardware: the measured-current form dropped the ceiling
from 690 RPM to 550 RPM. Since Id_setpoint is always 0, only the d-axis term
survives in practice.

Note *why* this is needed, because it is not the usual reason. As a
DISTURBANCE the coupling is DC in the dq frame and the integrator rejects it
almost perfectly (~0.5 mA of error through a 2 s ramp), so deleting it looks
harmless at low speed and did in fact pass review once on that argument. It
matters because it couples the two regulators DYNAMICALLY: once we exceeds
BANDWIDTH the axes drive each other faster than either loop can respond. The
signature is unmistakable -- phase currents that are garbage while spinning
and a perfect sine the instant the rotor is stopped and we goes to zero.

Implement a tight loop with proper ADC sampling timing, proper center-aligned PWM, etc.

## Voltage limit, current derate, and the speed ceiling

On a 12 V bus this drive runs out of VOLTAGE long before it runs out of
current, and ud is what consumes it:

    ud = -we * Lq * iq          (uq measured only 0.8 V at 300 RPM, and falling)

The only lever on ud is iq, so the correct response to running out of voltage
is to command LESS CURRENT, not to throw torque away. Each control cycle:

    iq_max = UD_FRAC * vbus / (|we| * Lq_online)      UD_FRAC = 0.85

This trades current for speed automatically, so the drive rides the voltage
limit rather than falling off it, and no hand-tuned current-per-speed table is
needed.

**The limit must be feedforward from speed, not feedback from measured ud.**
piStep already clamps its output to +-vbus, so |ud| can never be *observed*
exceeding the bus and a feedback version can never derate by more than
(1 - UD_FRAC). Measured on hardware: a feedback derate targeting 0.85 of the
bus actually delivered 0.995 and sat pinned at the backstop continuously.

A backstop still clamps |u| to vbus, scaling ud and uq TOGETHER so the applied
vector keeps its angle (an earlier version zeroed uq outright, which removed
all torque at exactly the speed where torque was needed and turned the ceiling
into a cliff). It must also pull the PI integrators down by the same factor:
piStep's anti-windup only sees its own per-axis +-vbus limit and cannot
observe the later scaling.

**The |u| <= vbus circle is correct here, not conservative.** Two independent
H-bridges do give a square limit reaching sqrt(2)*vbus on the diagonals, but
for a ROTATING vector va = |u|*cos(theta+phi) sweeps the full amplitude every
cycle, so |va| <= vbus requires |u| <= vbus. The square only helps a
stationary vector. The remaining lever on voltage is overmodulation (the
square-wave fundamental reaches 4/pi * vbus, ~27% more, at the cost of
deliberate harmonics), not a different clamp shape.

Measured behaviour: speed and current trade directly, rpm_max * iq ~= 280 at
0.5 A rising to ~370 at 1.0 A (the product is not constant because Lq
saturates). Lower current is monotonically faster until torque runs out;
below ~0.3 A the motor is torque-limited instead. With the derate active the
drive reached **1700 RPM**, against ~400 RPM before the parameters were
corrected.

## Control delay compensation

Current is sampled at the top of the ISR; the duty computed from it is applied
at the NEXT ISR for that motor and then held for a full tick, so the voltage
lands ~1.5 ticks after the measurement. At 700 RPM that is 125 us = 26
electrical degrees of stale angle, which rotates the applied vector backwards
and visibly distorts the phase currents. The inverse Park therefore runs at
theta_e + we*COMP_TICKS*dt, while the forward Park stays at the angle the
sample was actually taken at.

## Control-rate limit

The commanded field advances in steps of
(rpm/60)*360*POLE_PAIRS/(PWM_HZ/2) electrical degrees -- i.e. **rpm/40** at
24 kHz. FOC generally wants 20+ updates per electrical cycle; at 1700 RPM
there are only 8.5, which is why the current cannot look like a clean sine up
there regardless of voltage headroom. The only lever is PWM_HZ, traded against
ISR budget.

## Bringup
- The board will be brought up in phases (so make some test programs as a sub-folder that reference the library in the exterior folder that can be deployed to the board, can use development environments but it shouldn't interfere with the libraries use as a library)

### Phase 1: Basic hardware
Enable & disable motor drivers, log current sensing, change RGB LED color when stop/go pressed, etc. - no motion, just reading the sensors and ensuring basic hardware functionality

### Phase 2: Open-loop control
Do open-loop SVPWM with a voltage such that the phase current is 0.5A and spin the motors at 60RPM open-loop. Still log phase current and let it be graphed via arduino serial plotter. Should show up as two sine waves

(Originally spec'd at 0.1A -- raised to 0.5A after bringup showed 0.1A doesn't give
enough torque to overcome this motor's detent/static friction, so it just buzzes in
place instead of rotating even with a correct startup ramp. See Known Pitfalls.)

### Phase 3: Simple closed-loop FOC
Enable the PI controllers, constant velocity, on BOTH motors. Runs one long
profile whose cruise phase dominates, so the axis spends minutes at speed.

Because there is no position sensor, the loop park-transforms the measured
current onto the COMMANDED angle -- which is false at power-up, when the real
rotor angle is arbitrary. The velocity ramp from zero resolves this on its
own: for its first moments the commanded field is barely moving while already
carrying full current, which is exactly an alignment hold, and the rotor is
captured long before the speed is high enough to outrun it. A separate
explicit align stage was tried and removed as redundant.

Holding a stationary field at a known current is still the right way to check
the current-sense GAIN, since with the rotor still there is no back-EMF and no
di/dt and uq reduces to Ohm's law (`I_actual * R + V_offset`). That comparison
is what verified this board's sense chain to 0.3%, and it is worth repeating
on any new board before trusting a current number -- phase 5's alignment stage
does exactly this as its first step.

Logs loop rate, worst-case loop dt, electrical degrees per field step, bus
voltage min/max, and per-motor sensA/sensB/ud/uq/id/iq/derate.

### Phase 4: Full library
Trapezoidal position profile to 100 rotations on BOTH motors, peak 400 RPM,
via `motorSetProfile` (position authoritative + velocity feedforward).
Exercises the decoupling, the voltage derate and the delay compensation
together under acceleration.

### Phase 5: R / Ld / Lq characterization
Measures the motor parameters the datasheet gets wrong. Open-loop voltage
(`focSetVoltageAB`, no PI in the path) plus the existing current sense.

- **R** -- 6-point V/I sweep regressed as I vs V. Using the SLOPE rather than
  any single V/I point makes it immune to current-sense offset and to the
  driver's fixed dead-time drop; both land in the intercept, which is reported
  separately as a diagnostic.
- **Ld** -- step response with the rotor parked by DC current on phase B,
  which makes B the d-axis so the step produces zero torque and the rotor
  cannot move. Fitted by weighted least squares on ln(residual) vs t: the
  SLOPE carries tau, so the (variable, ~125 us) pipeline delay lands in the
  intercept and cancels. Weighting by r^2 and cutting the window at r > 0.15
  removes the Jensen bias of the log transform; 16 averaged captures take the
  spread to ~0.2%. Verified in simulation before being written to firmware.
- **Lq** -- CANNOT be measured at standstill: any q-axis current makes torque
  and the rotor moves, and it cannot be restrained by hand (the locked-rotor
  attempt returned R^2 0.95 with R_step 19% high). Measured instead on the
  SPINNING machine, where id is held at 0 and ud = -we*Lq*iq is a straight
  line through a 100-300 RPM sweep. R^2 0.9999.

Self-checks printed: R from the sweep vs R from the step (should agree to
~1%), fit R^2 for each, both regression intercepts (should be ~0), per-point
id during the speed sweep (off-zero means lost sync), and a raw CSV dump of
the averaged step captures so the exponential can be eyeballed.

Run from cold: the routine dissipates enough to warm the winding, and R
measurably tracks temperature at +0.39%/degC.

Document library use properly in README.md. Do everything with the consideration that this repository will be published as a platformio library.

## Known Pitfalls (learned during bringup)

These are hard-won findings from real hardware debugging. Read before touching PWM/GPIO init code.

### `pwm_set_mask_enabled()` overwrites the WHOLE PWM enable register
`pwm_hw->en` is a single register shared by every PWM slice on the chip (not per-slice). The
pico-sdk's `pwm_set_mask_enabled(mask)` does `pwm_hw->en = mask` — a blind overwrite, not a masked
set. If you build a mask containing only "the slices I care about" and call this, you will
silently **disable every other slice already running**, including the LED's PWM slices (GPIO
8/9 share slice 4, GPIO 14 is slice 7) if `ledInit()` ran first. This exact bug caused the RGB LED
to go stuck full-brightness ("white") the instant `focStart()`/`pwmInit()` ran, even though the
LED code itself was untouched and correct — it looked like an LED bug for a long time but was a
motor-PWM-init bug with an LED-shaped symptom. Always OR your mask into the existing register
(`pwm_hw->en |= mask;`) instead of overwriting it, so unrelated slices are left alone.

### GPIO-to-PWM-slice aliasing is real and easy to miss
Slice number = `(gpio >> 1) & 7` (for gpio < 32), channel = `gpio & 1`. Two GPIOs 16 apart, or an
even/odd pair like 8/9, can land on the *same* slice as different channels. `PIN_LED_R` (8) and
`PIN_LED_G` (9) are one such pair. `pwm_init()` resets the whole slice's counter-compare register
(both channels) every time it's called — so if two aliased pins are each `pwm_init`'d
independently, the second call clobbers the first pin's compare value. Init each unique slice
exactly once; only touch channel levels afterward via `pwm_set_gpio_level`/`pwm_set_chan_level`
(masked read-modify-write), never by re-running `pwm_init` on an already-configured slice.

### Diagnosing "firmware says X but hardware shows Y": read the actual registers
When a symptom contradicts what the code should be doing, don't keep guessing at the source —
print the raw peripheral registers (e.g. `pwm_hw->slice[N].csr`, `.cc`) directly over serial. This
is what actually found the enable-register bug above, after multiple plausible-sounding
software theories (LED slice aliasing, brownout/reset loops, hardware damage) were all
individually ruled out by hardware evidence and would have kept wasting time otherwise.

### `gpio_set_function(pin, GPIO_FUNC_PWM)` silently overrides an earlier SIO setup
`cfgSlice()`/`cfgEn()`-style helpers that call `gpio_set_function(pin, GPIO_FUNC_PWM)` change
*that specific pin's* mux, independent of anything `hwInit()` did earlier (`gpio_init` +
`gpio_set_dir` + `gpio_put`). During the first locked-antiphase experiment, a helper was reused
for its slice-init side effect on a pin that was supposed to stay a static GPIO output (motor 1's
EN pin, meant to be held HIGH) — but the helper also flipped that pin's own function to PWM with
its compare register left at 0 (permanent 0% duty), silently overriding the earlier `gpio_put(...,
1)`. Symptom: driver outputs stayed Hi-Z (disabled) the whole time — felt like inert detent
"braking" under hand rotation, not actual holding torque, with current telemetry reading ~0. Fix:
never assume a pin keeps whatever GPIO function `hwInit()` gave it — if a later init step touches
that pin's slice for any reason, explicitly re-assert the function/level you actually want
afterward, or (better, as done in the final design) restructure so nothing ever needs to touch a
pin's function twice with conflicting intents.

### A `volatile` struct field that is never copied out reads as stack garbage
`controlStep()` snapshots the shared `Setpoint` into a local under the spinlock,
one field at a time. Two fields (`ab_mode`, `vb_duty`) were never assigned — and
because the local is uninitialised, `if (sp.ab_mode)` was testing whatever was on
the stack. It consistently read false, so the direct-duty branch never ran and
execution silently fell through to the closed-loop PI, which then interpreted
`iq_cmd` as an amps command when the caller had stored a *duty* there. Every
caller of that mode commanded zero current and got zero current: phase 1c had
never worked as designed, and phase 5 initially reported `R = -72785 ohm`
because every point in its sweep measured no current at all. The user-visible
symptoms — nothing felt on the shaft, no supply draw — were the diagnosis; the
numbers were just an honest reading of a winding carrying nothing. Worth noting
this also invalidated the `ISENSE_SCALE` sign calibration, which had been
performed with phase 1c. When adding a field to a struct that is snapshotted
field-by-field, add the copy in the same commit, and prefer initialising the
local so a missed field fails loudly rather than plausibly.

### The setpoint refresh rate, not the ISR rate, quantizes the commanded field
The control ISR can only act on the angle the caller last handed it, so the
commanded field advances in steps of `(rpm/60)*360*POLE_PAIRS*T_caller`. A
`delayMicroseconds(500)` in the bringup loop meant 528 us per iteration, which
at 500 RPM is a **79 degree** electrical jump per update — a quarter cycle at a
time, not a rotating vector. It was plainly audible as clicking and roughness,
and it eats pull-out margin because the instantaneous load angle swings by half
the step. Deleting one delay line took the step to the ISR-rate floor of 12.5
degrees and the motor went "basically silent". Note the winding is a low-pass
(tau = L/R ~ 1 ms) so the *current* still looks like a decent sine while the
*command* is a coarse staircase — the scope will not show you this.

### The current waveform cannot tell you whether the rotor is turning
With no position sensor, the loop parks measured current onto the COMMANDED
angle, so the regulator produces a textbook rotating current vector whether the
rotor follows or sits dead still. It looks *better* when stalled, because a
stationary rotor generates no back-EMF and `uq` only has to supply `R*iq`. A
perfect sine at 400 RPM is therefore fully consistent with a stalled motor. Use
`uq` as the discriminator: back-EMF exists only if the rotor is moving, so a `uq`
near `R*iq` means stalled regardless of how clean the phase currents look. Hours
were spent debugging a current loop that was working correctly the whole time.

### Unbounded float angle accumulation stalls silently as the loop gets faster
`theta` accumulated without wrapping. Float resolution scales with magnitude, so
once theta passes ~5000 rad the per-iteration increment (2.6e-4 rad at 500 RPM
with a 5 us loop) vanishes into rounding entirely and the motor quietly stops
advancing. It was safe only because the loop was slow enough that the increment
was 100x larger — speeding the loop up is exactly what triggers it. `POLE_PAIRS`
is an integer, so wrapping mechanical angle at 2*PI is exactly 50 whole
electrical cycles and leaves `electricalAngle()` bit-for-bit unchanged. Phase 4's
profile position needs `double` for the same reason: its 628 rad target is where
float resolution (6e-5) approaches one iteration's increment.

### A clamped signal cannot be used as the feedback for a limiter
A current derate was built as feedback on `|ud|`, scaling the command by
`ud_lim/|ud|`. It could not work: `piStep` already clamps its output to
`+-vbus`, so `|ud|` never *exceeds* the bus and the ratio could never drop below
`UD_FRAC` — structurally incapable of derating more than 15% no matter how far
over budget the drive really was. Symptom: at 0.8 A it delivered 0.68 A where
~0.29 A was needed, and the ceiling barely moved. Before closing a loop around a
measurement, check whether anything upstream saturates it.

### Clamping after the regulator winds up integrators it cannot see
The `|u| <= vbus` backstop scales `ud`/`uq` *after* `piStep`, whose anti-windup
only knows its own per-axis limit. With `ud = -12.10` inside `+-12.16` the PI
believed it was unsaturated and kept integrating against a ceiling it had no way
to observe, while the applied voltage sat pinned at `|u| = vbus` continuously.
Any post-hoc actuator clamp has to be reported back to the regulator.

### Feedforward decoupling must use the command, not the measurement
See the FOC section: measured-current decoupling closes a loop whose gain
approaches exactly 1 at speed, and it dropped the measured ceiling from 690 RPM
to 550 RPM. This is a real trap because the textbook equations are written in
terms of Id/Iq and it is natural to reach for the measured values.

### Trust the datasheet for nothing you can measure
Phase resistance was 16% off (3.5 vs 4.04 ohm) and inductance was wrong by more
than 2x, in a direction that halved the current-loop bandwidth and doubled the
inductive voltage the loop fights at speed. The datasheet also never mentioned
that the machine is 2.2:1 salient, so a single inductance constant was wrong for
one axis no matter what value it held. Correcting these two numbers and
splitting the gains per axis took the ceiling from ~400 RPM to 560 RPM with no
hardware change; the rest of the path to 1700 RPM followed from measuring what
those parameters do under load.

### `Serial` is USB CDC on this core and blocks on 1 ms USB frames
The `115200` passed to `Serial.begin` is ignored. Writes block on USB frame
scheduling, so a telemetry print costs ~500-1000 us in bursts. When the caller
also owns the commanded angle, that stall lands directly in the motor as a
periodic angle jump — audible as regular clicking at the print rate (40 Hz for a
25 ms interval). Instrumentation that measures worst-case loop time will mostly
be measuring its own printing. Keeping the angle inside the ISR makes the
control path immune to it.

### Both ISENSE_SCALE values negative is a 180 degree frame rotation
`ISENSE_SCALE_A` and `ISENSE_SCALE_B` are both `-1`, which negates the whole
alpha-beta vector. That is a rotation, not a reflection, so it is harmless for
running the motor — torque is unaffected and the rotor simply sits 180
electrical degrees from where the model labels it — but it inverts the sign of
the back-EMF term and is why every `lambda_m` extraction returns negative. A
single flipped channel would be a reflection instead, which turns the dq
quantities into a 2*we signal the PI cannot track; the fact that `|i|` tracks its
command is what rules that out.

### Disabling a PWM slice freezes its output level; CC/CTR writes do not clear it
The PWM output flop only re-evaluates on a counter clock edge, and a slice with its `en` bit
cleared gets no clocks. So clearing `pwm_hw->en` leaves the pin latched at whatever level it
happened to hold, and the obvious follow-up — `pwm_set_gpio_level(pin, 0)` plus
`pwm_set_counter(slice, 0)` — does **nothing** to the pin, because neither write is a clock edge.
`buzzOff()` did exactly that and left BUZZ stuck high on roughly half of all calls (50% duty), a
steady 3.3V across the piezo, measurable with a multimeter after the sketch finished. The piezo is
capacitive so it draws no meaningful current, but the DC bias couples 3V3 rail ripple straight
across the diaphragm — heard as continuous hiss/static long after the last note — and the
diaphragm dumps its stored charge as an audible chirp when the board is powered down. The only way
to guarantee 0V is to take the pin away from the PWM block: `gpio_set_function(pin, GPIO_FUNC_SIO)`
with the pad driven low, and hand it back to `GPIO_FUNC_PWM` in `buzzOn()` only after the slice is
already toggling.
