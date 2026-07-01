#include "foc.h"
#include "pwm.h"
#include "adc.h"
#include "hw.h"
#include "debug.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/multicore.h"

namespace rotev {

struct Setpoint { float theta_mech; float iq_cmd; bool enabled; };

static volatile Setpoint s_sp[2]   = {{0,0,false},{0,0,false}};
static volatile AB       s_tel[2]  = {{0,0},{0,0}};
static PIState  s_pid[2], s_piq[2];
static OmegaEst s_omega[2];
static volatile bool s_lagcomp = false;
static spin_lock_t* s_lock;
static volatile int s_turn = 0; // 0 -> motor1, 1 -> motor2

// per-motor pin sets
static void phasePins(Motor m, uint32_t& enA, uint32_t& phA, uint32_t& enB, uint32_t& phB) {
  if (m == MOTOR_1) { enA=PIN_ENA_1; phA=PIN_PHA_1; enB=PIN_ENB_1; phB=PIN_PHB_1; }
  else              { enA=PIN_ENA_2; phA=PIN_PHA_2; enB=PIN_ENB_2; phB=PIN_PHB_2; }
}

static void controlStep(Motor m) {
  Setpoint sp;
  bool lag;
  uint32_t irq = spin_lock_blocking(s_lock);
  sp.theta_mech = s_sp[m].theta_mech;
  sp.iq_cmd     = s_sp[m].iq_cmd;
  sp.enabled    = s_sp[m].enabled;
  lag           = s_lagcomp;
  spin_unlock(s_lock, irq);

  uint32_t enA,phA,enB,phB; phasePins(m, enA,phA,enB,phB);

  if (!sp.enabled) {
    piReset(s_pid[m]); piReset(s_piq[m]); omegaReset(s_omega[m]);
    pwmSetPhase(enA, phA, 0.0f); pwmSetPhase(enB, phB, 0.0f);
    return;
  }

  const float dt = 1.0f / (PWM_HZ / 2.0f); // 12 kHz per motor

  AB i = adcSampleMotor(m);                       // measured phase currents (alpha=A, beta=B)
  float theta_e = electricalAngle(sp.theta_mech);
  float we = omegaStep(s_omega[m], theta_e, dt, 0.05f);

  DQ dq = park(i, theta_e);                        // open-loop position assumption
  float uq = piStep(s_piq[m], sp.iq_cmd - dq.q, KP, KI, dt, VBUS_V);
  float ud = piStep(s_pid[m], 0.0f      - dq.d, KP, KI, dt, VBUS_V);
  if (lag) {
    uq += we * PHASE_L * dq.d;   // +we*Ld*Id
    ud -= we * PHASE_L * dq.q;   // -we*Lq*Iq
  }

  AB v = inversePark(ud, uq, theta_e, VBUS_V);     // normalized duties [-1,1]
  pwmSetPhase(enA, phA, v.a);
  pwmSetPhase(enB, phB, v.b);

  uint32_t irq2 = spin_lock_blocking(s_lock);
  s_tel[m].a = i.a;
  s_tel[m].b = i.b;
  spin_unlock(s_lock, irq2);
}

static void __not_in_flash_func(pwmWrapISR)() {
  debugTimingHigh();
  pwm_clear_irq(pwmMasterSlice());
  Motor m = (s_turn == 0) ? MOTOR_1 : MOTOR_2;
  controlStep(m);
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
  for (int m = 0; m < 2; ++m) { piReset(s_pid[m]); piReset(s_piq[m]); omegaReset(s_omega[m]); }
  multicore_launch_core1(core1Entry);
}

void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].iq_cmd = clampCurrent(iq_cmd);
  s_sp[m].enabled = enabled;
  spin_unlock(s_lock, irq);
}

AB focTelemetry(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  AB t; t.a = s_tel[m].a; t.b = s_tel[m].b;
  spin_unlock(s_lock, irq);
  return t;
}

void focSetLagComp(bool on) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_lagcomp = on;
  spin_unlock(s_lock, irq);
}

} // namespace rotev
