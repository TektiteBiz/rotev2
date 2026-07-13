#include "foc.h"
#include <cmath>
#include "pwm.h"
#include "adc.h"
#include "adc_ext.h"
#include "hw.h"
#include "debug.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

namespace rotev {

struct Setpoint { float theta_mech; float iq_cmd; float vb_duty; bool enabled; bool openloop; bool ab_mode; };

static volatile Setpoint s_sp[2]   = {{0,0,0,false,false,false},{0,0,0,false,false,false}};
static volatile AB       s_tel[2]  = {{0,0},{0,0}};
static PIState  s_pid[2], s_piq[2];
static spin_lock_t* s_lock;
static volatile int s_turn = 0; // 0 -> motor1, 1 -> motor2
// Pre-computed duties applied at the very start of the next ISR invocation,
// before any slow computation, to minimize CC-register latency from the PWM wrap.
static float s_next_a[2] = {0, 0};
static float s_next_b[2] = {0, 0};

// per-motor pin sets
static void phasePins(Motor m, uint32_t& phA, uint32_t& phB) {
  if (m == MOTOR_1) { phA=PIN_PHA_1; phB=PIN_PHB_1; }
  else              { phA=PIN_PHA_2; phB=PIN_PHB_2; }
}

// i is pre-sampled at the start of the ISR, before any pwmSetPhase call,
// so both ADC channels land within ~8 µs of counter=TOP (well before the
// LOW→HIGH switching edge whose timing depends on duty cycle).
static void __not_in_flash_func(controlStep)(Motor m, AB i) {
  float vbus = adcExtVbus();
  Setpoint sp;
  uint32_t irq = spin_lock_blocking(s_lock);
  sp.theta_mech = s_sp[m].theta_mech;
  sp.iq_cmd     = s_sp[m].iq_cmd;
  sp.enabled    = s_sp[m].enabled;
  sp.openloop   = s_sp[m].openloop;
  spin_unlock(s_lock, irq);

  if (!sp.enabled) {
    piReset(s_pid[m]); piReset(s_piq[m]);
    s_next_a[m] = 0.0f; s_next_b[m] = 0.0f;
    return;
  }

  const float dt = 1.0f / (PWM_HZ / 2.0f); // 12 kHz per motor

  float theta_e = electricalAngle(sp.theta_mech);
  float ud, uq;

  if (sp.ab_mode) {
    // Direct stationary-frame duty mode: bypass all transforms and the PI.
    // Used by phase 1c to test current sensing without dq-frame complexity.
    piReset(s_pid[m]); piReset(s_piq[m]);
    auto clamp1 = [](float x){ return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x); };
    s_next_a[m] = clamp1(sp.iq_cmd);
    s_next_b[m] = clamp1(sp.vb_duty);
    uint32_t irq2 = spin_lock_blocking(s_lock);
    s_tel[m].a = i.a; s_tel[m].b = i.b;
    spin_unlock(s_lock, irq2);
    return;
  }

  if (sp.openloop) {
    // Voltage mode: iq_cmd holds uq directly (volts), ud=0, PI not touched.
    ud = 0.0f;
    uq = sp.iq_cmd;
  } else {
    DQ dq = park(i, theta_e);
    uq = piStep(s_piq[m], sp.iq_cmd - dq.q, KP, KI, dt, vbus);
    ud = piStep(s_pid[m], 0.0f      - dq.d, KP, KI, dt, vbus);
    // ud and uq are each bounded individually by piStep, but their vector sum
    // can exceed the bus voltage circle; inversePark's per-phase duty clamp
    // then clips ud/uq non-proportionally, distorting the dq split.
    // Prioritize ud (holds id at 0) over uq: letting id drift nonzero would
    // add uncontrolled phase current beyond IMAX_A, so derate iq instead.
    float ud_mag = fabsf(ud);
    if (ud_mag > vbus) {
      ud *= vbus / ud_mag;
      uq = 0.0f;
    } else {
      float uq_budget = sqrtf(vbus * vbus - ud * ud);
      if (fabsf(uq) > uq_budget) uq = (uq < 0.0f ? -uq_budget : uq_budget);
    }
  }

  AB v = inversePark(ud, uq, theta_e, vbus);       // normalized duties [-1,1]
  s_next_a[m] = v.a;
  s_next_b[m] = v.b;

  uint32_t irq2 = spin_lock_blocking(s_lock);
  s_tel[m].a = i.a;
  s_tel[m].b = i.b;
  spin_unlock(s_lock, irq2);
}

static void __not_in_flash_func(pwmWrapISR)() {
  debugTimingHigh();
  pwm_clear_irq(pwmMasterSlice());
  Motor m = (s_turn == 0) ? MOTOR_1 : MOTOR_2;
  // Locked-antiphase: EN is held HIGH continuously (hardwired) and current
  // is always actively driven (never coasting), so there's no "freewheeling
  // window" in the old sign-magnitude sense -- counter=0 is still a valid,
  // well-defined sample point (the fixed trough of the center-aligned
  // triangular carrier).
  AB i_meas = adcSampleMotor(m);
  // Apply last cycle's pre-computed duty.
  uint32_t phA, phB;
  phasePins(m, phA, phB);
  pwmSetPhase(phA, s_next_a[m]);
  pwmSetPhase(phB, s_next_b[m]);
  controlStep(m, i_meas);  // computes s_next_a/b for the cycle after this one
  s_turn ^= 1;
  debugTimingLow();
}

static void core1Entry() {
  debugTimingInit();
  irq_set_exclusive_handler(PWM_IRQ_WRAP, pwmWrapISR);
  pwm_set_irq_enabled(pwmMasterSlice(), true);
  irq_set_enabled(PWM_IRQ_WRAP, true);   // serviced on core1 (this core)
  while (true) __wfi();
}

void focStart() {
  s_lock = spin_lock_init(spin_lock_claim_unused(true));
  pwmInit();
  adcInit();
  for (int m = 0; m < 2; ++m) { piReset(s_pid[m]); piReset(s_piq[m]); }
  multicore_launch_core1(core1Entry);
}

void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].iq_cmd = clampCurrent(iq_cmd);
  s_sp[m].enabled = enabled;
  s_sp[m].openloop = false;
  spin_unlock(s_lock, irq);
}

void focSetVoltage(Motor m, float theta_mech, float uq_volts, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].iq_cmd = uq_volts;   // repurposed as voltage command; no current clamping
  s_sp[m].enabled = enabled;
  s_sp[m].openloop = true;
  spin_unlock(s_lock, irq);
}

AB focTelemetry(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  AB t; t.a = s_tel[m].a; t.b = s_tel[m].b;
  spin_unlock(s_lock, irq);
  return t;
}

void focSetVoltageAB(Motor m, float va_duty, float vb_duty, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].iq_cmd  = va_duty;
  s_sp[m].vb_duty = vb_duty;
  s_sp[m].enabled = enabled;
  s_sp[m].ab_mode = true;
  spin_unlock(s_lock, irq);
}

} // namespace rotev
