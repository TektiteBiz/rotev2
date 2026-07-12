#pragma once
#include <cstdint>

namespace rotev {

enum Motor : uint8_t { MOTOR_1 = 0, MOTOR_2 = 1 };
enum Button : uint8_t { BTN_STOP = 0, BTN_GO = 1 };

// --- Motor 14HS11-1004 ---
constexpr int   POLE_PAIRS = 50;      // 1.8 deg/step -> 200 steps/rev
constexpr float PHASE_R    = 3.5f;    // ohms
constexpr float PHASE_L    = 0.0035f; // H (Ld = Lq)

// --- Control (PI pole placement) ---
constexpr float BANDWIDTH = 1000.0f;              // rad/s
constexpr float KP        = BANDWIDTH * PHASE_L;   // 3.5
constexpr float KI        = BANDWIDTH * PHASE_R;   // 3500

// --- Current sense (INA186A3, 15 mohm, REF 1.65V, 3.3V ADC) ---
constexpr float SHUNT_OHMS    = 0.015f;
constexpr float INA_GAIN      = 100.0f;
constexpr float ISENSE_REF_V  = 1.65f;
constexpr float ADC_VREF      = 3.3f;
constexpr float ADC_MAX       = 4095.0f;
constexpr float VOLTS_PER_AMP = SHUNT_OHMS * INA_GAIN; // 1.5
constexpr float IMAX_A        = 1.1f;                  // command clamp / sensor range

// --- Per-channel calibration (ISENSE-CAL knob) ---
// Measure at motor idle (no current): ISENSE_OFFSET_x = -(ADC reading at 0 A).
// Measure at known current: ISENSE_SCALE_x = expected_A / measured_A.
// sensA is accurate; sensB shows -0.125 A offset and ~1.5x gain — tune these.
constexpr float ISENSE_OFFSET_A = 0.0f;
constexpr float ISENSE_OFFSET_B = 0.0f;
constexpr float ISENSE_SCALE_A  = 1.0f;
constexpr float ISENSE_SCALE_B  = 1.0f;

// --- PWM / loop ---
constexpr uint32_t PWM_HZ        = 24000;
// PWM output polarity is inverted (cfgEn): the active-HIGH phase is centred
// around counter=TOP; counter=0 (IRQ) is in the stable freewheeling window.
// ADC samples (8 µs total) start at IRQ and finish well before the first edge.
constexpr int      ADC_OVERSAMPLE = 2;

// --- Bus voltage (future: from ADC) ---
constexpr float VBUS_V = 12.0f;

// --- GPIO map ---
constexpr uint32_t PIN_ENA_2 = 0,  PIN_PHA_2 = 1,  PIN_ENB_2 = 2,  PIN_PHB_2 = 3;
constexpr uint32_t PIN_ENA_1 = 4,  PIN_PHA_1 = 5,  PIN_ENB_1 = 6,  PIN_PHB_1 = 7;
constexpr uint32_t PIN_LED_R = 8,  PIN_LED_G = 9,  PIN_LED_B = 14;
constexpr uint32_t PIN_BTN_STOP = 19, PIN_BTN_GO = 20;
constexpr uint32_t PIN_NSLEEP_1 = 21, PIN_NSLEEP_2 = 22;
constexpr uint32_t PIN_SOB_1 = 26, PIN_SOA_1 = 27, PIN_SOB_2 = 28, PIN_SOA_2 = 29;
constexpr uint32_t ADC_SOB_1 = 0, ADC_SOA_1 = 1, ADC_SOB_2 = 2, ADC_SOA_2 = 3;
constexpr uint32_t PIN_LOOP_TIMING = 10; // SPI1 SCK; bringup only

} // namespace rotev
