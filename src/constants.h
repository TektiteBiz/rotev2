#pragma once
#include <cstdint>

namespace rotev {

enum Motor : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };

// --- Motor 14HS11-1004 ---
constexpr int POLE_PAIRS = 50;      // 1.8 deg/step -> 200 steps/rev
// R and L are MEASURED on hardware (bringup phase 5), not datasheet values.
// The datasheet's 3.5 ohm / 3.5 mH were both wrong, and this motor turns out
// to be strongly salient -- Lq is 2.2x Ld -- so a single inductance cannot
// describe both axes.
// Values are the mean of two motors, since constants.h is global and both
// axes run off it. They agreed closely -- R within 1.6%, Ld 1.5%, Lq 3.6%,
// saliency 2.20 vs 2.16 -- so averaging halves the worst-case error to under
// 2%. For scale, Lq itself moves ~25% with current on a SINGLE motor from
// saturation, so unit-to-unit spread is the smaller effect by far.
//     motor 1: R 4.0744  Ld 3.8231 mH  Lq 8.4265 mH
//     motor 2: R 4.0089  Ld 3.7660 mH  Lq 8.1263 mH
//   R  : 6-point V/I sweep regressed as I vs V, so the slope excludes the
//        driver's dead-time drop and any sense offset (both land in the
//        intercept). Cross-checked three ways on motor 1: sweep 4.074, Ld
//        step response 4.091, alignment hold 4.26 minus the 0.07 V offset.
//   Ld : step response with the rotor parked on phase B, which makes B the
//        d-axis so the step produces no torque and the rotor cannot move.
//        R^2 0.998+, and repeatable across runs on one motor to 0.01%.
//   Lq : slope of ud vs (we*iq) over a 100-300 RPM sweep on the running
//        machine. Standstill cannot measure Lq -- any q-axis current makes
//        torque and the rotor moves. R^2 0.9999, good to roughly +-5% (the
//        load angle leaks a little back-EMF into ud).
//        NOTE the runtime does not lean on this number: the derate uses a
//        per-motor ONLINE Lq estimate (s_lq in foc.cpp) that tracks each
//        motor's actual operating point and saturation. PHASE_LQ only seeds
//        it and sets KP_Q and the decoupling feedforward, all of which
//        tolerate a few percent easily.
// Ld < Lq is expected here: the magnet sits in the d-axis flux path with
// mu_r ~ 1, so the d-axis is the high-reluctance one.
//
// CONFIRMED: at 0.5 A the drive tops out at 560 RPM on a 12.12 V bus, which
// back-solves to Lq = 8.27 mH -- 1.9% from the measured 8.43. The ceiling is
// where |ud| = we*Lq*iq reaches the bus, since ud is by far the largest term
// (uq measured only 0.8 V at 300 RPM and falling). That gives
//        rpm_max * iq ~= 280        on a 12 V bus
// so speed and current trade directly, and the only levers on the ceiling
// are less current, more bus, or a smaller Lq.
//
// CAVEAT: Lq is measured at 0.5 A only, and iron saturates, so Lq(I) is
// expected to fall at higher current -- which would make the high-current
// ceiling better than the relation above predicts and leave KP_Q over-gained
// there. Not yet measured; sweep LQ_AMPS in bringup phase 5 to find out.
constexpr float PHASE_R  = 4.0417f;     // ohms, mean of 2 (datasheet: 3.5)
constexpr float PHASE_LD = 0.0037946f;  // H, mean of 2 (datasheet: 3.5 mH)
constexpr float PHASE_LQ = 0.0082764f;  // H, mean of 2 -- sets ud at speed

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
// ud = -we*Lq*iq grows with BOTH speed and current, and on a 12 V bus it is
// what runs out first (uq measured only 0.8 V at 300 RPM and falling). The
// only lever on ud is iq, so when ud starts eating the bus the right answer
// is to command less current, not to throw away torque. Trading current for
// speed this way lets the drive ride the voltage limit instead of falling
// off it, and it self-tunes: the derate is driven by the ud the PI actually
// produces, so it needs no speed estimate and no Lq value -- which matters,
// because Lq falls with current and is not well characterised above 0.5 A.
constexpr float UD_FRAC = 0.85f;    // of the bus that ud may consume
constexpr float IQ_MIN = 0.05f;     // floor on the derated current command

// --- Control delay compensation ---
// Current is sampled at the top of the ISR, the duty computed from it is
// applied at the NEXT ISR for this motor (+1 tick) and then held for a whole
// tick, so the voltage lands ~1.5 ticks after the measurement -- 26 electrical
// degrees at 700 RPM, which rotates the applied vector backwards and visibly
// distorts the phase currents. The inverse Park therefore runs at
// theta_e + we*COMP_TICKS*dt while the forward Park stays at the angle the
// sample was actually taken at.
//
// 2.5, not the 1.5 the pipeline alone suggests: velocity mode used to advance
// its angle before using it, handing itself an undocumented extra tick of
// lead, and every result up to 1700 RPM was obtained with that in place. The
// pre-increment is gone (foc.cpp reads before advancing, so both command
// modes now mean "angle now"), so that tick is carried here instead, where it
// applies to position and profile commands too. Retune from here if the
// control rate changes.
constexpr float COMP_TICKS = 2.5f;

// Cross-coupling decoupling strength. 1.0 = full, 0.0 = off (set 0 to A/B
// test it -- it is not yet established that it helps this machine).
constexpr float DECOUPLE_FRAC = 1.0f;
constexpr float WE_ALPHA = 0.02f;   // speed-estimator LPF, position mode only
constexpr float LQ_ALPHA = 0.001f;  // online Lq estimator LPF (~80 ms)
constexpr float WE_TIMEOUT_S = 0.05f;  // no angle change for this long -> stopped
constexpr uint32_t PROF_AGE_MAX_US = 50000;  // cap velocity extrapolation
// Acceleration feedforward in the profile extrapolation. Set 0 to fall back
// to first order. At normal caller rates the whole extrapolation is a
// sub-degree effect, so if changing this alters behaviour noticeably, the
// caller is stalling for milliseconds and THAT is the thing to fix.
constexpr float PROF_ACC_FF = 1.0f;
// Profile tracking gain, per control tick. The ISR integrates the commanded
// velocity on its own tick (jitter-free, exactly as velocity mode does) and
// pulls that integral toward the caller's authoritative angle at this rate.
// Small enough to filter the caller's timing quantization, fast enough that
// real position error is gone in ~50 ms: tau = CTRL_DT / PROF_TRACK.
constexpr float PROF_TRACK = 0.005f;

// --- Current sense (INA181A2, 30 mohm, REF 1.65V, 3.3V ADC) ---
constexpr float SHUNT_OHMS = 0.03f;
constexpr float INA_GAIN = 50.0f;
constexpr float ISENSE_REF_V = 1.65f;
constexpr float ADC_VREF = 3.3f;
constexpr float ADC_MAX = 4095.0f;
constexpr float VOLTS_PER_AMP = SHUNT_OHMS * INA_GAIN;  // 1.5
constexpr float IMAX_A = 1.1f;  // command clamp / sensor range

// --- Per-channel calibration (ISENSE-CAL / ISENSE-SIGN knob) ---
// Measure at motor idle (no current): ISENSE_OFFSET_x = -(ADC reading at 0 A).
// Measure at known current: ISENSE_SCALE_x = expected_A / measured_A.
// sensA's ADC-reported sign is inverted relative to the sign convention
// inversePark()'s output duty assumes: confirmed on hardware by commanding a
// known-positive phase-A voltage (phase1c) and reading back negative current.
// Invisible in open-loop (Phase 2 never reads current back to decide output)
// but fatal in closed-loop (Phase 3): park()/piStep() feed the mismatched
// sign back through the same theta used to command output, turning negative
// feedback into positive feedback and driving straight to saturation
// regardless of ramp/alignment. sensB has not been sign-tested independently.
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
// ADC samples (8 µs total) start at IRQ and finish well before the first edge.
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
