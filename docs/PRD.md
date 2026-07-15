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

## ADIS1015 ADC
On I2C1 (GPIO 18/19) there is an ADS1015IRUGR ADC. AIN0 on this ADC is connected to bus voltage with a voltage divider with 7.3kohm on the high side and 2.2kohm on the low side. This needs to be sampled at ~1khz for the FOC loop inverse park. AIN1-3 are exposed to the user so consider how to display them to the user too.

# FOC
High frequency FOC will be implemented and run. The specifications are:
- 24kHz PWM frequency (with 12kHz control loop for each motor, alternating between motor 1 and motor 2)
- Need to be able to disable the control and each driver via nSLEEP (e.g. motorEnable(MOTOR_1) type of thing)
- Need to be able to write current and theta to each motor (e.g. motorWrite(100, 1, MOTOR_1) for 100 rad, 1 amp)

Implementation-wise, there must be
- Center aligned PWM for SVPWM
- No clarke transform is needed since the currents are already in alpha-beta frame
- For park transform, assume the rotor is at the target position, and run PI control
- Run PI with pole placement as follows:
kP = BANDWIDTH * INDUCTANCE
kI = BANDWIDTH * RESISTANCE

The motor specs are follows (write this in README):
- Part No: 14HS11-1004
- Step Angle: 1.8deg
- Phase Resistance: 3.5ohms
- Inductance: 3.5mH

Use the resistance and inductance for the PI tuning above. 
Use bandwidth = ~160 Hz (so 1000 rad/s)

Additionally, do lag compensation:

Ud_compensation = −ωe · Lq · Iq
Uq_compensation = +ωe · Ld · Id

So in the end:
Uq = PIq(Iq_setpoint − Iq_measured) + ωe·Ld·Id
Ud = PId(0 − Id_measured) − ωe·Lq·Iq

Implement a tight loop with proper ADC sampling timing, proper center-aligned PWM, etc.

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
Enable the PI controllers but don't worry about lag compensation, simply do basic FOC and allow setting the current and running S curves or simple constant velocity

### Phase 4: Full library
Implement lag compensation and test S curves up to 100 rotations and 300rpm

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
