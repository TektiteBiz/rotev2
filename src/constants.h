#pragma once
#include <cstdint>

namespace rotev {

enum Motor : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };

// --- Motor 14HS11-1004 ---
constexpr int POLE_PAIRS = 50;      // 1.8 deg/step -> 200 steps/rev
// R and L are MEASURED on hardware (bringup phase 5), not datasheet values:
// the datasheet's 3.5 ohm / 3.5 mH are both wrong, and this motor is strongly
// salient (Lq = 2.2 * Ld), so one inductance cannot describe both axes.
// Ld < Lq is expected: the magnet sits in the d-axis flux path with mu_r ~ 1,
// so the d-axis is the high-reluctance one.
//   R  : 6-point V/I sweep regressed as I vs V, so the slope excludes the
//        driver dead-time drop and the sense offset (both land in the
//        intercept). Cross-checked three ways within 0.5%.
//   Ld : step response with the rotor parked on phase B, which makes B the
//        d-axis so the step makes no torque and the rotor cannot move.
//        R^2 0.998+, repeatable to 0.01%.
//   Lq : slope of ud vs (we*iq) over a 100-300 RPM sweep on the running
//        machine -- standstill cannot measure it, since any q-axis current
//        makes torque. R^2 0.9999, good to about +-5%.
// Values below are the mean of two motors (they agreed to within R 1.6%,
// Ld 1.5%, Lq 3.6%), which halves the worst-case error since constants.h is
// global and both axes run off it:
//     motor 1: R 4.0744  Ld 3.8231 mH  Lq 8.4265 mH
//     motor 2: R 4.0089  Ld 3.7660 mH  Lq 8.1263 mH
// The runtime does not lean on PHASE_LQ for the derate: that uses a per-motor
// ONLINE estimate (s_lq in foc.cpp) which follows each motor's saturation
// curve. PHASE_LQ only seeds it and sets KP_Q and the decoupling feedforward.
//
// Speed/current trade: the drive runs out of ud = -we*Lq*iq long before uq
// (uq measured 0.8 V at 300 RPM and falling), so the ceiling is where |ud|
// reaches the bus. Measured at 0.5 A: 560 RPM on a 12.12 V bus, i.e.
//        rpm_max * iq ~= 280        on a 12 V bus
// The only levers on it are less current, more bus, or a smaller Lq. Lq is
// characterised at 0.5 A only and iron saturates, so Lq(I) should fall at
// higher current, making the high-current ceiling better than that predicts.
constexpr float PHASE_R  = 4.0417f;     // ohms, mean of 2 (datasheet: 3.5)
constexpr float PHASE_LD = 0.0037946f;  // H, mean of 2 (datasheet: 3.5 mH)
constexpr float PHASE_LQ = 0.0082764f;  // H, mean of 2 -- sets ud at speed

// Fixed q-axis current for every closed-loop (profile) MOVE. Not exposed: the
// trade above makes current a tuning knob for the drive, not for the user.
// Standstill uses MOTOR_HOLD_AMPS instead -- see below.
//
// 0.9 A is bounded by TOUCH TEMPERATURE, not by the motor. Dissipation is
// |i|^2 * R_winding (sinusoidal drive: the two phases carry cos and sin, so
// the sum of squares is constant -- NOT 2*|i|^2*R).
//
// R_winding is NOT PHASE_R. PHASE_R = 4.0417 is winding + Rds(on) + shunt, and
// the driver's ~0.23 ohm share heats the PCB, not the motor. Winding alone is
// ~3.81 ohm (PHASE_R minus the ~0.23 ohm driver share), vs 3.5 datasheet cold
// value. Using the datasheet 3.5 here UNDERSTATES motor heating by 9%.
//
// At 3.81 ohm: 0.9 A -> 3.09 W -> ~50 C surface at 25 C ambient, which is
// slightly OVER the ~48 C IEC 62368-1 prolonged-contact limit for metal.
// 0.85 A would land on it. This is only acceptable because the standstill hold
// below cuts the AVERAGE well under the limit -- a continuous-motion duty cycle
// would not be covered. MEASURE the surface before shipping; the thermal
// resistance here is a datasheet figure, and mounting dominates it.
//
// Also note the current sense: the INA181 specifies gain error only for
// V_OUT in [0.5, V_S-0.5], which at 1.5 V/A is +-0.767 A. 0.9 A is outside
// that window, so gain error at the current peaks is unspecified -- it shows
// up as waveform distortion, i.e. torque ripple. Fix that before going higher.
constexpr float MOTOR_AMPS = 0.9f;

// --- Control (PI pole placement, per axis) ---
// Each axis places its controller zero on its own plant pole: the d-axis
// plant is 1/(Ld*s + R) with a pole at -R/Ld, so the zero must sit at
// KI/KP_D = R/Ld, which falls out of KP_D = BW*Ld with a SHARED KI = BW*R.
// The same holds for q. So KP differs per axis and KI does not.
constexpr float BANDWIDTH = 1000.0f;         // rad/s, both axes
constexpr float KP_D = BANDWIDTH * PHASE_LD; // 3.79
constexpr float KP_Q = BANDWIDTH * PHASE_LQ; // 8.28
constexpr float KI   = BANDWIDTH * PHASE_R;  // 4042

// --- Voltage-limited torque derate ---
// ud = -we*Lq*iq grows with BOTH speed and current, and it is what runs out
// first on a 12 V bus. The only lever on ud is iq, so when ud starts eating
// the bus the right answer is to command less current, not to throw away
// torque: that lets the drive ride the voltage limit instead of falling off
// it. See foc.cpp for why the limit is computed from speed, not measured ud.
// UD_FRAC is the FEEDFORWARD half of the limiter. It is derived, not guessed:
// ud and uq add in QUADRATURE, so the real constraint is |u| <= k*vbus, i.e.
//        UD_FRAC <= sqrt(k^2 - (R*I/vbus)^2)
// Reserving k = 0.95 for PI control authority, the worst point is the derate
// corner (full MOTOR_AMPS, so the largest uq = R*I): that gives 0.91, rising
// to 0.95 at 600 RPM where the derate has cut I and uq with it. 0.91 is the
// safe value across the range. The previous 0.85 was a flat linear reserve
// applied to a quadratic constraint, which stranded 11-15% of the bus above
// the corner -- and was simultaneously too SMALL at the corner, surviving only
// because of the quadrature addition.
// NOTE the two R's. For the VOLTAGE budget the right R is the full series
// PHASE_R (4.0417) -- winding + Rds(on) + shunt all drop volts. For the THERMAL
// budget below it is the WINDING alone, since the driver's share heats the PCB.
// The reserve is for uq, so it must be sized against the uq that actually
// occurs -- and uq is NOT R*I. MEASURED on hardware above 400 RPM: uq mean
// 4.83 V, max 7.29 V, against an R*I model value of 2.74 V. The model
// understates by ~2 V because uq also carries we*lam*sin(gamma), the
// load-angle term, which the no-load model omits entirely.
//
//     UD_FRAC <= sqrt(k^2 - (uq/vbus)^2),  k = 0.95 for PI authority
//   at 10.5 V:  R*I model -> 0.913   measured mean -> 0.831   max -> 0.648
//
// 0.83 is sized on the measured mean, leaving the trim to absorb excursions
// toward the max. The earlier 0.88/0.91 were derived from the R*I model and
// were therefore over-committed: on the validation run the backstop fired
// often enough to hold the trim below 1.0 for 56% of the move.
//
// A fixed fraction is still the wrong shape here -- the right budget is
// sqrt((k*vbus)^2 - uq^2) using the MEASURED uq from the previous tick, which
// needs no constant at all. Left as future work: it closes a loop through the
// load angle and wants bench time before it ships.
constexpr float UD_FRAC = 0.83f;    // of the bus that ud may consume
constexpr float IQ_MIN = 0.05f;     // floor on the derated current command

// --- Saturation trim (the FEEDBACK half of the limiter) ---
// The feedforward above knows speed but nothing else: not load angle, vehicle
// mass, grade, battery sag, winding temperature, or per-unit R spread (the
// datasheet allows R +-10%, L +-20%, and per-unit constants are not an option
// on a fleet). The trim closes that gap by riding the ACTUAL voltage limit.
//
// Trigger: loss of d-axis regulation. The PI holds id at 0 until the bus runs
// out; then the |u| backstop scales ud down, the d-axis loop cannot get the
// voltage it asked for, and id drifts. That is what saturation looks like from
// inside the loop, and it is independent of WHY the bus ran out.
//
// This is NOT the feedback derate rejected in PRD.md: that one scaled by
// (ud_lim/|ud|), and piStep already clamps to +-vbus so the ratio could never
// drop below UD_FRAC -- structurally blind. id has no such ceiling; the
// further over budget you are, the larger the signal.
//
// Hysteresis, not sluggishness, is what prevents limit-cycling -- so recovery
// can be fast (~300 ms) instead of the seconds a plain integrator would need.
// A whole profile is 10-20 s, so a 2 s recovery would waste a fifth of a move.
constexpr float TRIM_MIN       = 0.70f;  // trim only ever cuts, never boosts
// PROPORTIONAL to how far over SAT_TRIP the duty is, not a fixed rate. With a
// fixed 12/s the 10 ms filter takes ~11 ms to decay back under the trip point
// even if the cause vanishes instantly, so the MINIMUM possible cut was 0.13 --
// 43% of the whole band -- and the loop limit-cycled at ~6 Hz. Proportional
// action self-limits: a marginal trip makes a marginal cut.
constexpr float TRIM_DOWN_RATE = 12.0f;  // /s at full saturation duty
constexpr float TRIM_UP_RATE   = 1.0f;   // /s -- full band in ~300 ms
constexpr float SAT_TRIP       = 0.10f;  // backstop duty above this -> cut
constexpr float SAT_CLEAR      = 0.02f;  // and below this (AND id clear) -> recover
// 0.05 A matches ID_TOL, the off-zero threshold phase 5 already found workable.
// It sits above the sense noise floor (~20-50 mA: 9.2 ENOB ADC + INA offset)
// and above the ~1 ms cross-coupling kick the d-axis loop takes after a fast
// iq change, which the filter below rejects.
// |id| is deliberately NOT a derate trigger -- see foc.cpp. It rises on loss
// of sync as well as on voltage saturation, and cutting current during a stall
// deepens it. Callers wanting a sync check should read motorCurrentD().
// The s_lq estimator's own validity floor. Was sharing IQ_MIN, which silently
// coupled it to the derate floor; they are unrelated quantities.
constexpr float LQ_EST_MIN_IQ  = 0.05f;
// The estimator's own backstop-duty ceiling. Numerically equal to SAT_CLEAR
// today, but they are different questions -- "is it safe to learn from this
// tick" vs "is it safe to give current back" -- and sharing a constant would
// silently couple them, which is the exact failure mode UNJUSTIFIED_CONSTANTS.md
// documents for IQ_MIN.
constexpr float LQ_EST_MAX_SAT = 0.02f;
constexpr float SAT_FILT_TAU_S = 0.010f; // 10 ms on backstop duty

// --- Standstill hold ---
// focEnable() promotes an axis with no profile to a standstill hold, and a
// finished move holds too, so without this the motor burns full MOTOR_AMPS
// while parked -- which on a robot is most of the duty cycle. Holding only has
// to beat detent plus drivetrain friction (plus any grade you must hold on),
// which is far less than moving torque.
//
// This is a TOUCH-TEMPERATURE product: dissipation is |i|^2 * R_winding, so
// 0.8 A is 2.47 W and ~45 C surface at 25 C ambient, already close to the
// ~48 C IEC 62368-1 prolonged-contact limit for metal. Cutting the hold to
// 0.4 A drops hold dissipation 4x and is what buys back the thermal budget.
// Tune DOWN until the axis backdrives under your worst holding load.
constexpr float MOTOR_HOLD_AMPS   = 0.4f;
// ASYMMETRIC on purpose. Slewing DOWN into the hold can be gentle -- nothing
// needs the torque. Slewing UP must be fast, because it happens at the START
// of a move, which is exactly when peak acceleration torque is demanded.
// Measured on hardware with a symmetric 5 A/s: the first 150 ms of every move
// ran at reduced current while ramping 0.4 -> 0.9 A, cutting torque during the
// acceleration ramp. 40 A/s puts the rise inside one 12.5 ms profile segment.
// Dwell before engaging the hold. st.done goes true the instant a leg ends,
// so a back-to-back sequence drops the current command between legs -- measured
// on hardware: 0.900 -> 0.567 -> 0.900 at rpm=0 between legs, a 37% torque cut
// during a handover. Requiring the axis to be done for HOLD_DWELL_S before
// engaging means a new profile arriving promptly never sees the hold at all.
constexpr float HOLD_DWELL_S = 0.25f;
// How long an invalid vbus reading may be papered over with VBUS_V before the
// axis stops driving rather than guess at the rail.
constexpr float VBUS_FAULT_S = 0.25f;
constexpr float HOLD_SLEW_UP_A_PER_S   = 40.0f;  // 0.4 -> 0.9 A in ~12 ms
constexpr float HOLD_SLEW_DOWN_A_PER_S = 5.0f;   // 0.9 -> 0.4 A in 100 ms

// --- Control delay compensation ---
// Current is sampled at the top of the ISR, the duty computed from it is
// applied at the NEXT ISR for this motor (+1 tick) and then held for a whole
// tick, so the voltage lands ~1.5 ticks after the measurement -- 26 electrical
// degrees at 700 RPM, which rotates the applied vector backwards and visibly
// distorts the phase currents. The inverse Park therefore runs at
// theta_e + we*COMP_TICKS*dt while the forward Park stays at the angle the
// sample was actually taken at.
// 2.5, not the 1.5 the pipeline alone accounts for: everything up to 1700 RPM
// was validated with one extra tick of lead present elsewhere, and this is the
// single place all phase advance now lives. Retune from here if PWM_HZ moves.
constexpr float COMP_TICKS = 2.5f;

// Cross-coupling decoupling strength. 1.0 = full, 0.0 = off (set 0 to A/B
// test it -- it is not yet established that it helps this machine).
constexpr float DECOUPLE_FRAC = 1.0f;
constexpr float LQ_ALPHA = 0.001f;  // online Lq estimator LPF (~80 ms)

// --- Current sense (INA181A2, 30 mohm, REF 1.65V, 3.3V ADC) ---
constexpr float SHUNT_OHMS = 0.03f;
constexpr float INA_GAIN = 50.0f;
constexpr float ISENSE_REF_V = 1.65f;
constexpr float ADC_VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f;
constexpr float VOLTS_PER_AMP = SHUNT_OHMS * INA_GAIN;  // 1.5
constexpr float IMAX_A = 1.1f;  // current-sense full-scale range

// --- Per-channel calibration (ISENSE-CAL / ISENSE-SIGN knob) ---
// Measure at motor idle (no current): ISENSE_OFFSET_x = -(ADC reading at 0 A).
// Measure at known current: ISENSE_SCALE_x = expected_A / measured_A.
// sensA's ADC-reported sign is inverted relative to the sign convention
// inversePark()'s output duty assumes: confirmed on hardware by commanding a
// known-positive phase-A voltage (phase1c) and reading back negative current.
// Invisible open-loop, fatal closed-loop -- park()/piStep() feed the
// mismatched sign back through the same theta used to command output, turning
// negative feedback into positive feedback and driving straight to saturation.
// sensB has not been sign-tested independently.
constexpr float ISENSE_OFFSET_A = 0.0f;
constexpr float ISENSE_OFFSET_B = 0.0f;
constexpr float ISENSE_SCALE_A = -1.0f;
constexpr float ISENSE_SCALE_B = -1.0f;

// --- PWM / loop ---
constexpr uint32_t PWM_HZ = 24000;
// Locked-antiphase drive (see Drivers section, PRD.md): EN is hardwired HIGH
// on all 4 drivers (pull-up, no GPIO), PH is PWMed at duty=(1+d)/2. Current
// is always actively driven (never coasting), so there's no freewheeling
// window; counter=0 (IRQ) is still the correct ADC sample point since it's
// the fixed trough of the center-aligned carrier, symmetric with counter=TOP.
// ADC samples (8 us total) start at IRQ and finish well before the first edge.
constexpr int ADC_OVERSAMPLE = 2;

// --- Bus voltage (startup fallback; live value comes from the ADS1015, see adc_ext.*) ---
constexpr float VBUS_V = 12.0f;

// --- GPIO map ---
// PHA_2/PHB_2 share PWM slice 0 (channels A/B); PHA_1/PHB_1 share slice 1.
// GPIO4-7 are unused by the driver (previously EN, now hardwired) and free.
constexpr uint32_t PIN_PHA_2 = 0, PIN_PHB_2 = 1, PIN_PHA_1 = 2, PIN_PHB_1 = 3;
constexpr uint32_t PIN_LED_R = 8, PIN_LED_G = 9, PIN_LED_B = 14;
constexpr uint32_t PIN_BTN_STOP = 20, PIN_BTN_GO = 21;
constexpr uint32_t PIN_NSLEEP_1 = 22, PIN_NSLEEP_2 = 23;
constexpr uint32_t PIN_SOB_1 = 26, PIN_SOA_1 = 27, PIN_SOB_2 = 28,
                   PIN_SOA_2 = 29;
constexpr uint32_t ADC_SOB_1 = 0, ADC_SOA_1 = 1, ADC_SOB_2 = 2, ADC_SOA_2 = 3;
constexpr uint32_t PIN_LOOP_TIMING = 10;  // SPI1 SCK; bringup only

// --- Buzzer (passive piezo, GPIO4) ---
constexpr uint32_t PIN_BUZZ = 4;
constexpr uint16_t BUZZ_MIN_HZ = 1000, BUZZ_MAX_HZ = 4000;

// --- ADS1015 external ADC (I2C1, GPIO18/19) ---
constexpr uint32_t PIN_I2C1_SDA = 18, PIN_I2C1_SCL = 19;
constexpr float ADS1015_FSR_V = 4.096f;          // PGA full-scale range used for all channels
constexpr float VBUS_DIV_HIGH_OHMS = 7300.0f;    // bus-voltage divider high side
constexpr float VBUS_DIV_LOW_OHMS = 2200.0f;     // bus-voltage divider low side

enum AdcChannel : uint8_t { ADC_AIN1 = 0, ADC_AIN2 = 1, ADC_AIN3 = 2 };

}  // namespace rotev
