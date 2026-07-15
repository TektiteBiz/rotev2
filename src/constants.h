#pragma once
#include <cstdint>

namespace rotev {

enum Motor : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };

// --- Motor 14HS11-1004 ---
constexpr int POLE_PAIRS = 50;      // 1.8 deg/step -> 200 steps/rev
constexpr float PHASE_R = 3.5f;     // ohms
constexpr float PHASE_L = 0.0035f;  // H (Ld = Lq)

// --- Control (PI pole placement) ---
constexpr float BANDWIDTH = 1000.0f;       // rad/s
constexpr float KP = BANDWIDTH * PHASE_L;  // 3.5
constexpr float KI = BANDWIDTH * PHASE_R;  // 3500

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
