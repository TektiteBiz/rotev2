#include "rotev.h"
#include "rotev_internal.h"
#include "hw.h"
#include "led.h"
#include "button.h"
#include "foc.h"
#include "buzz.h"
#include "adc_ext.h"

namespace rotev {

void begin() {
  hwInit();
  ledInit();
  buzzInit();
  adcExtInit();
  // adcExtInit() must run before focStart(): focStart() launches core1,
  // which immediately starts calling adcExtVbus() every control cycle, and
  // that call needs adc_ext's spinlock/cache already initialized.
  focStart();
}

void motorEnable(Motor m, bool enable) {
  if (enable) {
    hwSetNsleep(m, true);
    focEnable(m, true);
  } else {
    // nSLEEP tri-states the bridge immediately; the ISR clears the duty on
    // its next pass for this motor, which is up to 167 us later.
    focEnable(m, false);
    hwSetNsleep(m, false);
  }
}

void motorSetProfile(Motor m, const Profile& p) { focSetProfile(m, p); }
Profile      motorProfile(Motor m)  { return focProfile(m); }
ProfileState motorProgress(Motor m) { return focProgress(m); }

void  buzzerOn(uint16_t freq_hz) { buzzOn(freq_hz); }
void  buzzerOff() { buzzOff(); }
void  ledColor(uint8_t r, uint8_t g, uint8_t b) { ledSet(r, g, b); }
bool  buttonPressed(Button b) { return buttonRead(b); }
float adcRead(AdcChannel ch) { return adcExtUser(ch); }
float busVoltage() { return adcExtVbus(); }

void motorSetVoltage(Motor m, float theta_rad, float uq_volts) {
  focSetVoltage(m, theta_rad, uq_volts);
}

void motorSetVoltageAB(Motor m, float va_volts, float vb_volts) {
  // Scale by the LIVE bus, not the nominal constant -- the FOC path has used
  // adcExtVbus() since the ADS1015 landed, and leaving this on VBUS_V would
  // make "volts" here mean something different from volts everywhere else.
  // Matters for phase 5, which infers R and L from the commanded voltage.
  float vbus = busVoltage();
  if (vbus <= 0.0f) vbus = VBUS_V;
  focSetVoltageAB(m, va_volts / vbus, vb_volts / vbus);
}

void motorSetVelocity(Motor m, float vel_rad_s) { focSetVelocity(m, vel_rad_s); }

float motorCurrentA(Motor m) { return focTelemetry(m).a; }
float motorCurrentB(Motor m) { return focTelemetry(m).b; }
float motorCurrentD(Motor m) { return focTelemetryI(m).d; }
float motorCurrentQ(Motor m) { return focTelemetryI(m).q; }
float motorVoltageD(Motor m) { return focTelemetryU(m).d; }
float motorVoltageQ(Motor m) { return focTelemetryU(m).q; }
float motorDerateTrim(Motor m) { return focDerateTrim(m); }
float motorCurrentCmd(Motor m) { return focCurrentCmd(m); }
float motorLqEstimate(Motor m) { return focLqEstimate(m); }
bool  motorFault(Motor m) { return focFault(m); }

} // namespace rotev
