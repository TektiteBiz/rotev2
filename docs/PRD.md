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
- 0: ENA_2
- 1: PHA_2
- 2: ENB_2
- 3: PHB_2
- 4: ENA_1
- 5: PHA_1
- 6: ENB_1
- 7: PHB_1
- 8: LED.R
- 9: LED.G
- 14: LED.B
- 19: BTN_STOP
- 20: BTN_GO
- 21: nSLEEP_1
- 22: nSLEEP_2
- 26: SOB_1
- 27: SOA_1
- 28: SOB_2
- 29: SOA_2

## Drivers
There are two stepper motors (motor 1 and motor 2). Each motor has phases A and B. Each phase has a DRV8874 motor driver set up with PH/EN mode, and a current sensor which is a INA186A3 with a 15mohm shunt and with REF biased to 1.65V. For example, ENB_2 is the EN pin on the B phase driver for motor 2. SOB_1 is the current output of the B phase of motor 1.

nSLEEP is connected between each pair, so both drivers for motor 1 share nSLEEP_1 and both drivers for motor 2 share nSLEEP_2

## Buttons
The buttons go HIGH when pressed, so need the internal pull-down. 

## RGB LED
The RGB LED will be controlled via PWM with the 3 pins. 

## Additional GPIOs
GPIOs 10, 11, 12, and 13 are meant for use by the user, but will also be used during bringup (explained later). These pins are the SPI1 bus, which must be documented in the readme. GPIOs 16 and 17 are I2C0 and feature I2C pull-ups on the board and are also exposed (but not used for bringup) and also must be documented in README.md. These pins serve as busses for the user but can also be used as general GPIO and not exclusively for SPI/I2C. The mappings when being used as a bus are as follows:
- 10: SCK
- 11: MOSI
- 12: MISO
- 13: CS
- 16: SDA
- 17: SCL

# FOC
High frequency FOC will be implemented and run. The specifications are:
- 24kHz PWM frequency (with 12kHz control loop for each motor, alternating between motor 1 and motor 2)
- Need to be able to disable the control and each driver via nSLEEP (e.g. motorEnable(MOTOR_1) type of thing)
- Need to be able to write current and theta to each motor (e.g. motorWrite(100, 1, MOTOR_1) for 100 rad, 1 amp)

Implementation-wise, there must be
- Center aligned PWM for SVPWM
- No clarke transform is needed since the currents are already in alpha-beta frame
- For park transform, assume the rotor is at the target position, and run PI control
- For now assume 12V as the bus voltage for inverse park, in the future there will be an ADC though
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
Do open-loop SVPWM with a voltage such that the phase current is 0.1A and spin the motors at 60RPM open-loop. Still log phase current and let it be graphed via arduino serial plotter. Should show up as two sine waves

### Phase 3: Simple closed-loop FOC
Enable the PI controllers but don't worry about lag compensation, simply do basic FOC and allow setting the current and running S curves or simple constant velocity

### Phase 4: Full library
Implement lag compensation and test S curves up to 100 rotations and 300rpm

For the current 12V assumption make the code for inverse park correctly encapsulated so fixing it to use the ADC does not need changes in multiple spots.

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
