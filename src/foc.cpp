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

struct Setpoint { float theta_mech; float iq_cmd; float vb_duty; float vel; uint32_t seq; bool enabled; bool openloop; bool ab_mode; bool velmode; bool profmode; };

static volatile Setpoint s_sp[2]   = {{0,0,0,0,0,false,false,false,false,false},{0,0,0,0,0,false,false,false,false,false}};
static volatile AB       s_tel[2]  = {{0,0},{0,0}};
// Applied dq voltage and measured dq current, for bringup telemetry. Written
// post-clamp so it reflects what actually reached inversePark().
static volatile DQ       s_telu[2] = {{0,0},{0,0}};
static volatile DQ       s_teli[2] = {{0,0},{0,0}};
static PIState  s_pid[2], s_piq[2];
static float    s_derate[2] = {1.0f, 1.0f};  // applied iq_cmd scaling, telemetry
static float    s_we[2] = {0.0f, 0.0f};      // electrical speed, rad/s
static float    s_prev_te[2] = {0.0f, 0.0f};
static bool     s_we_primed[2] = {false, false};
static float    s_lq[2] = {PHASE_LQ, PHASE_LQ};  // online Lq, tracks saturation
static uint32_t s_hold[2] = {0, 0};              // ticks since theta_e moved
static uint32_t s_seq[2] = {0, 0};               // last profile seq seen
static uint32_t s_age[2] = {0, 0};               // ticks since that seq arrived
static spin_lock_t* s_lock;
static volatile int s_turn = 0; // 0 -> motor1, 1 -> motor2
// Pre-computed duties applied at the very start of the next ISR invocation,
// before any slow computation, to minimize CC-register latency from the PWM wrap.
static float s_next_a[2] = {0, 0};
static float s_next_b[2] = {0, 0};

// One control period per motor: the wrap IRQ fires at PWM_HZ and alternates
// motors, so each motor is serviced at PWM_HZ/2.
static constexpr float CTRL_DT = 1.0f / (PWM_HZ / 2.0f);
static constexpr float TWO_PI_F = 6.28318530717958647692f;
static constexpr float PI_F = 3.14159265358979323846f;

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
  // Velocity mode: advance the commanded angle here, on the hardware-timed
  // control tick, instead of trusting core0 to do it. core0 stalls (USB CDC
  // writes, blocking I2C, anything) would otherwise show up directly as
  // angle steps in the commanded field -- i.e. as torque ripple. Wrapping at
  // one mechanical revolution is exact because POLE_PAIRS is an integer, and
  // it keeps theta small enough that float resolution stays far below the
  // per-tick increment.
  if (s_sp[m].velmode && s_sp[m].enabled) {
    float th = s_sp[m].theta_mech + s_sp[m].vel * CTRL_DT;
    if (th >= TWO_PI_F) th -= TWO_PI_F;
    else if (th < 0.0f) th += TWO_PI_F;
    s_sp[m].theta_mech = th;
  }
  sp.theta_mech = s_sp[m].theta_mech;

  // Profile mode: position is authoritative, velocity only fills the gap
  // since it arrived. Integrating velocity alone (velmode) is fine for "spin
  // at N RPM", but for a position profile the caller's integral and the
  // ISR's would be two independent sums of the same velocity and would drift
  // apart with nothing to correct them. Here every new setpoint resets the
  // angle outright, so error cannot accumulate; the feedforward only smooths
  // over core0 being late, and is capped so a wedged core0 cannot run away.
  if (s_sp[m].profmode && s_sp[m].enabled) {
    if (s_sp[m].seq != s_seq[m]) { s_seq[m] = s_sp[m].seq; s_age[m] = 0; }
    else if (s_age[m] < PROF_AGE_MAX) ++s_age[m];
    sp.theta_mech += s_sp[m].vel * ((float)s_age[m] * CTRL_DT);
  }
  sp.iq_cmd     = s_sp[m].iq_cmd;
  sp.enabled    = s_sp[m].enabled;
  sp.openloop   = s_sp[m].openloop;
  // These two were missing: `sp` is an uninitialized local, so testing
  // sp.ab_mode below was reading stack garbage. It happened to read false,
  // which silently routed every ab_mode caller into the closed-loop PI with
  // va_duty reinterpreted as an amps command -- and dropped vb_duty on the
  // floor. Phases 1c and 5 both commanded nothing at all.
  sp.ab_mode    = s_sp[m].ab_mode;
  sp.vb_duty    = s_sp[m].vb_duty;
  sp.velmode    = s_sp[m].velmode;
  sp.profmode   = s_sp[m].profmode;
  sp.vel        = s_sp[m].vel;
  spin_unlock(s_lock, irq);

  if (!sp.enabled) {
    piReset(s_pid[m]); piReset(s_piq[m]); s_derate[m] = 1.0f;
    s_we[m] = 0.0f; s_we_primed[m] = false; s_lq[m] = PHASE_LQ; s_hold[m] = 0;
    s_age[m] = 0;
    s_next_a[m] = 0.0f; s_next_b[m] = 0.0f;
    return;
  }

  const float dt = CTRL_DT; // 12 kHz per motor

  float theta_e = electricalAngle(sp.theta_mech);

  // Electrical speed. In velocity mode the commanded value is exact and
  // noise-free; otherwise fall back to differencing theta_e, wrapping the
  // delta so the +-PI boundary does not read as a huge jump.
  float we;
  if (sp.velmode || sp.profmode) {
    we = sp.vel * (float)POLE_PAIRS;
    s_we[m] = we;
  } else if (!s_we_primed[m]) {
    s_prev_te[m] = theta_e; s_we_primed[m] = true; s_hold[m] = 0; we = 0.0f;
  } else {
    // In position mode theta_mech is written by core0, whose loop may run far
    // slower than this 12 kHz tick -- so theta_e arrives as a staircase, flat
    // for several ticks and then jumping. Differencing every tick would read
    // 0,0,0,0,huge and hand a violently spiky we to the derate, the
    // decoupling and the delay compensation all at once. Count the ticks the
    // angle sat still and divide by the time that actually elapsed instead.
    float d = theta_e - s_prev_te[m];
    if (d > PI_F) d -= TWO_PI_F; else if (d < -PI_F) d += TWO_PI_F;
    ++s_hold[m];
    if (fabsf(d) > 1e-6f) {
      s_we[m] += WE_ALPHA * (d / (s_hold[m] * CTRL_DT) - s_we[m]);
      s_prev_te[m] = theta_e;
      s_hold[m] = 0;
    } else if (s_hold[m] * CTRL_DT > WE_TIMEOUT_S) {
      s_we[m] = 0.0f;  // genuinely stopped, not just between updates
    }
    we = s_we[m];
  }

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
    s_teli[m].d = dq.d; s_teli[m].q = dq.q;
    // Voltage-limited current derate, FEEDFORWARD from speed.
    // The earlier feedback version measured |ud| and scaled by
    // (ud_lim/|ud|) -- which cannot work, because piStep already clamps its
    // output to +-vbus. |ud| therefore never exceeds the bus, the ratio never
    // drops below UD_FRAC, and the loop could not cut more than 15% no
    // matter how far over budget it really was. Computing the limit from
    // speed instead has no such blind spot:
    //     ud = -we*Lq*iq   =>   iq_max = UD_FRAC*vbus / (|we|*Lq)
    // It is instant, cannot saturate, and needs no loop dynamics.
    float we_mag = fabsf(we);
    float iq_eff = sp.iq_cmd;
    if (we_mag > 1.0f) {
      // s_lq, not PHASE_LQ: Lq falls with current, so a constant measured at
      // 0.5 A under-derates at the low currents the limiter itself produces.
      // Measured on hardware: targeting 0.85 of the bus actually delivered
      // 0.995 and sat pinned at the backstop.
      float iq_max = (UD_FRAC * vbus) / (we_mag * s_lq[m]);
      if (iq_max < IQ_MIN) iq_max = IQ_MIN;
      if (iq_eff >  iq_max) iq_eff =  iq_max;
      if (iq_eff < -iq_max) iq_eff = -iq_max;
    }
    s_derate[m] = (fabsf(sp.iq_cmd) > 1e-4f) ? iq_eff / sp.iq_cmd : 1.0f;

    // Separate KP per axis: this motor is salient (Lq = 2.2 * Ld), so one
    // gain cannot place both poles. KI is shared -- see constants.h.
    uq = piStep(s_piq[m], iq_eff - dq.q, KP_Q, KI, dt, vbus);
    ud = piStep(s_pid[m], 0.0f   - dq.d, KP_D, KI, dt, vbus);

    // Cross-coupling decoupling, driven by the COMMAND, not the measurement.
    //     Ld*did/dt = ud - R*id + we*Lq*iq
    //     Lq*diq/dt = uq - R*iq - we*Ld*id - we*lm
    // Cancelling the off-diagonal terms needs -we*Lq*iq on d and +we*Ld*id
    // on q. Using the MEASURED currents for that (as the original lag comp
    // did) is a trap: it closes a loop ud -> id -> uq -> iq -> ud whose gain
    // is (we*Lq)(we*Ld)/(Zd*Zq), and at high speed Zd -> we*Ld and
    // Zq -> we*Lq, so the gain approaches exactly 1 -- marginally stable
    // before the pipeline delay is even counted. It also amplifies
    // current-sense ripple by we*Lq, which is 30 V/A at 690 RPM. Measured on
    // hardware the ceiling dropped from 690 to 550 RPM.
    // The command has no such path. id_cmd is always 0, so only the d-axis
    // term survives.
    ud -= DECOUPLE_FRAC * we * PHASE_LQ * iq_eff;

    // Online Lq, from the voltage the loop DEMANDS before any clipping.
    // Holding id at 0 against a current vector rotating at we requires
    // exactly ud = -we*Lq*iq, so the demand divides straight back out to Lq
    // at the present operating current -- no separate characterisation, and
    // it follows the saturation curve on its own.
    if (we_mag > 1.0f && fabsf(dq.q) > 0.05f) {
      float lq_obs = fabsf(ud) / (we_mag * fabsf(dq.q));
      if (lq_obs > PHASE_LD && lq_obs < 4.0f * PHASE_LQ)
        s_lq[m] += LQ_ALPHA * (lq_obs - s_lq[m]);
    }

    // Backstop for whatever the derate has not caught up with yet. Scale ud
    // and uq TOGETHER so the applied voltage vector keeps its angle: the old
    // code zeroed uq outright, which removed all torque at exactly the speed
    // where torque was needed, and turned the ceiling into a cliff.
    float umag = sqrtf(ud * ud + uq * uq);
    if (umag > vbus) {
      float sc = vbus / umag;
      ud *= sc;
      uq *= sc;
      // piStep's anti-windup only sees its own per-axis +-vbus limit, and
      // this scaling happens afterwards -- so without pulling the
      // integrators down by the same factor they wind up against a ceiling
      // they cannot observe. Measured pinned at |u| = vbus continuously.
      s_pid[m].integ *= sc;
      s_piq[m].integ *= sc;
    }
  }

  // Apply at the angle the rotor will be at when the voltage actually lands,
  // not the angle the current was sampled at -- see COMP_TICKS.
  float theta_out = theta_e + we * (COMP_TICKS * CTRL_DT);
  AB v = inversePark(ud, uq, theta_out, vbus);     // normalized duties [-1,1]
  s_telu[m].d = ud; s_telu[m].q = uq;
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
  for (int m = 0; m < 2; ++m) { piReset(s_pid[m]); piReset(s_piq[m]); s_derate[m] = 1.0f; }
  multicore_launch_core1(core1Entry);
}

void focSetpoint(Motor m, float theta_mech, float iq_cmd, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].iq_cmd = clampCurrent(iq_cmd);
  s_sp[m].enabled = enabled;
  s_sp[m].openloop = false;
  s_sp[m].ab_mode = false;
  s_sp[m].velmode = false;
  s_sp[m].profmode = false;
  spin_unlock(s_lock, irq);
}

// Velocity mode: the caller supplies mechanical rad/s and the control ISR
// integrates it. theta_mech is left untouched here, so switching over from
// focSetpoint() continues from wherever that left the angle (e.g. 0 after an
// alignment hold) rather than jumping.
void focSetVelocity(Motor m, float vel_rad_s, float iq_cmd, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].vel = vel_rad_s;
  s_sp[m].iq_cmd = clampCurrent(iq_cmd);
  s_sp[m].enabled = enabled;
  s_sp[m].openloop = false;
  s_sp[m].ab_mode = false;
  s_sp[m].velmode = true;
  s_sp[m].profmode = false;
  spin_unlock(s_lock, irq);
}

// Profile mode: absolute position plus a velocity feedforward. theta_mech is
// applied exactly as given on every update, so the caller's profile -- not an
// ISR-side integral -- decides where the axis ends up.
void focSetProfile(Motor m, float theta_mech, float vel_rad_s, float iq_cmd,
                   bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].vel = vel_rad_s;
  s_sp[m].iq_cmd = clampCurrent(iq_cmd);
  s_sp[m].enabled = enabled;
  s_sp[m].openloop = false;
  s_sp[m].ab_mode = false;
  s_sp[m].velmode = false;
  s_sp[m].profmode = true;
  ++s_sp[m].seq;
  spin_unlock(s_lock, irq);
}

void focSetVoltage(Motor m, float theta_mech, float uq_volts, bool enabled) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].iq_cmd = uq_volts;   // repurposed as voltage command; no current clamping
  s_sp[m].enabled = enabled;
  s_sp[m].openloop = true;
  s_sp[m].ab_mode = false;
  s_sp[m].velmode = false;
  s_sp[m].profmode = false;
  spin_unlock(s_lock, irq);
}

float focDerate(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  float d = s_derate[m];
  spin_unlock(s_lock, irq);
  return d;
}

DQ focTelemetryU(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  DQ t; t.d = s_telu[m].d; t.q = s_telu[m].q;
  spin_unlock(s_lock, irq);
  return t;
}

DQ focTelemetryI(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  DQ t; t.d = s_teli[m].d; t.q = s_teli[m].q;
  spin_unlock(s_lock, irq);
  return t;
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
  s_sp[m].velmode = false;
  s_sp[m].profmode = false;
  spin_unlock(s_lock, irq);
}

} // namespace rotev
