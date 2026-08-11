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

// Fixed q-axis current for every closed-loop (profile) move. Not exposed:
// the trade above makes current a tuning knob for the drive, not for the
// user, and 0.8 A leaves headroom to 1700 RPM after the derate.
constexpr float MOTOR_AMPS = 0.8f;

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
constexpr float IMAX_A = 1.1f;  // command clamp / sensor range

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
