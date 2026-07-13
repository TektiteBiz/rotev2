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

void motorWriteVoltage(float theta_rad, float uq_volts, Motor m) {
  s_theta[m] = theta_rad;
  focSetVoltage(m, theta_rad, uq_volts, s_en[m]);
}

void motorWriteVoltageAB(float va_volts, float vb_volts, Motor m) {
  focSetVoltageAB(m, va_volts / VBUS_V, vb_volts / VBUS_V, s_en[m]);
}

void setLagComp(bool on) { focSetLagComp(on); }

void buzzerOn(uint16_t freq_hz) { buzzOn(freq_hz); }
void buzzerOff() { buzzOff(); }
float adcRead(AdcChannel ch) { return adcExtUser(ch); }
float busVoltage() { return adcExtVbus(); }

void ledColor(uint8_t r, uint8_t g, uint8_t b) { ledSet(r, g, b); }
bool buttonPressed(Button b) { return buttonRead(b); }
float motorCurrentA(Motor m) { return focTelemetry(m).a; }
float motorCurrentB(Motor m) { return focTelemetry(m).b; }

} // namespace rotev
