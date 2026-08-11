#include "rotev.h"
#include "hw.h"
#include "led.h"
#include "button.h"
#include "foc.h"
#include "buzz.h"
#include "adc_ext.h"

namespace rotev {

static float s_theta[2] = {0,0};
static float s_iq[2]    = {0,0};
static bool  s_en[2]    = {false,false};

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

void motorEnable(Motor m) {
  hwSetNsleep(m, true);
  s_en[m] = true;
  focSetpoint(m, s_theta[m], s_iq[m], true);
}

void motorDisable(Motor m) {
  s_en[m] = false;
  focSetpoint(m, s_theta[m], 0.0f, false);
  hwSetNsleep(m, false);
}

void motorWrite(float theta_rad, float amps, Motor m) {
  s_theta[m] = theta_rad;
  s_iq[m] = amps;
  focSetpoint(m, theta_rad, amps, s_en[m]);
}

void motorWriteVelocity(float vel_rad_s, float amps, Motor m) {
  s_iq[m] = amps;
  focSetVelocity(m, vel_rad_s, amps, s_en[m]);
}

void motorWriteProfile(float theta_rad, float vel_rad_s, float acc_rad_s2,
                       float amps, Motor m) {
  s_theta[m] = theta_rad;
  s_iq[m] = amps;
  focSetProfile(m, theta_rad, vel_rad_s, acc_rad_s2, amps, s_en[m]);
}

void motorWriteVoltage(float theta_rad, float uq_volts, Motor m) {
  s_theta[m] = theta_rad;
  focSetVoltage(m, theta_rad, uq_volts, s_en[m]);
}

void motorWriteVoltageAB(float va_volts, float vb_volts, Motor m) {
  // Scale by the LIVE bus, not the nominal constant -- the FOC path has used
  // adcExtVbus() since the ADS1015 landed, and leaving this on VBUS_V made
  // "volts" here mean something different from volts everywhere else. Matters
  // for phase 5, which infers R and L from the commanded voltage.
  float vbus = busVoltage();
  if (vbus <= 0.0f) vbus = VBUS_V;
  focSetVoltageAB(m, va_volts / vbus, vb_volts / vbus, s_en[m]);
}

void buzzerOn(uint16_t freq_hz) { buzzOn(freq_hz); }
void buzzerOff() { buzzOff(); }
float adcRead(AdcChannel ch) { return adcExtUser(ch); }
float busVoltage() { return adcExtVbus(); }

void ledColor(uint8_t r, uint8_t g, uint8_t b) { ledSet(r, g, b); }
bool buttonPressed(Button b) { return buttonRead(b); }
float motorDerate(Motor m) { return focDerate(m); }
float motorVoltageD(Motor m) { return focTelemetryU(m).d; }
float motorVoltageQ(Motor m) { return focTelemetryU(m).q; }
float motorCurrentD(Motor m) { return focTelemetryI(m).d; }
float motorCurrentQ(Motor m) { return focTelemetryI(m).q; }
float motorCurrentA(Motor m) { return focTelemetry(m).a; }
float motorCurrentB(Motor m) { return focTelemetry(m).b; }

} // namespace rotev
