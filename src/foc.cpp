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

enum Mode : uint8_t { MODE_OFF = 0, MODE_PROFILE, MODE_VOLTAGE, MODE_AB };

// Everything core0 hands to the control ISR, and the progress it hands back.
// Guarded by s_lock rather than volatile: the profile evaluation below has to
// read and write several fields as one consistent set.
struct Setpoint {
  Mode     mode;
  bool     enabled;
  float    theta_mech;   // commanded mechanical angle, [0, 2*pi)
  float    uq;           // MODE_VOLTAGE: q-axis volts
  float    va_duty;      // MODE_AB
  float    vb_duty;      // MODE_AB
  Profile  prof;         // MODE_PROFILE
  bool     cruise;       // MODE_PROFILE: ignore prof, hold cruise_vel forever
  float    cruise_vel;   // rad/s
  uint32_t ticks;        // control ticks since the profile started
  ProfileState prog;     // published progress
};

static Setpoint s_sp[2];
static volatile AB s_tel[2]  = {{0,0},{0,0}};
// Applied dq voltage and measured dq current, for bringup telemetry. Written
// post-clamp so it reflects what actually reached inversePark().
static volatile DQ s_telu[2] = {{0,0},{0,0}};
static volatile DQ s_teli[2] = {{0,0},{0,0}};
static PIState s_pid[2], s_piq[2];
static float   s_lq[2] = {PHASE_LQ, PHASE_LQ};  // online Lq, tracks saturation
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
// Largest tick count that (float)ticks still represents exactly.
static constexpr uint32_t TICKS_MAX = 1u << 24;

// per-motor pin sets
static void phasePins(Motor m, uint32_t& phA, uint32_t& phB) {
  if (m == MOTOR_1) { phA=PIN_PHA_1; phB=PIN_PHB_1; }
  else              { phA=PIN_PHA_2; phB=PIN_PHB_2; }
}

// i is pre-sampled at the start of the ISR, before any pwmSetPhase call,
// so both ADC channels land within ~8 us of counter=TOP (well before the
// LOW->HIGH switching edge whose timing depends on duty cycle).
static void __not_in_flash_func(controlStep)(Motor m, AB i) {
  float vbus = adcExtVbus();

  uint32_t irq = spin_lock_blocking(s_lock);
  s_tel[m].a = i.a;
  s_tel[m].b = i.b;
  Mode  mode    = s_sp[m].mode;
  bool  enabled = s_sp[m].enabled;
  // Read BEFORE advancing, so theta_mech always means "the angle right now".
  // All phase advance lives in COMP_TICKS, in one place.
  float theta   = s_sp[m].theta_mech;
  float uq_cmd  = s_sp[m].uq;
  float va_duty = s_sp[m].va_duty;
  float vb_duty = s_sp[m].vb_duty;
  float vel     = 0.0f;
  if (enabled && mode == MODE_PROFILE) {
    // The profile is executed here, on the hardware-timed control tick, not
    // by core0: a core0 stall (USB CDC writes, blocking I2C, anything) would
    // otherwise show up directly as a step in the commanded field, i.e. as
    // torque ripple. The elapsed time comes from an integer tick count, so a
    // multi-minute move cannot accumulate float summation drift.
    float t = (float)s_sp[m].ticks * CTRL_DT;
    ProfileState st;
    if (s_sp[m].cruise) {
      // pos from t, not a running sum: adding a fixed increment to a growing
      // float stalls outright once the increment falls below half an ulp
      // (at 200 rpm that is ~26 minutes, after which pos would freeze while
      // the shaft kept turning).
      st.t = t; st.pos = s_sp[m].cruise_vel * t;
      st.vel = s_sp[m].cruise_vel; st.acc = 0.0f; st.done = false;
    } else {
      st = s_sp[m].prof.at(t);
      st.t = t;
    }
    vel = st.vel;
    // Once the move is over the axis holds: velocity 0, angle frozen, still
    // energised. Freezing the counter also keeps t from running away.
    // Cruise has no end, so cap its counter rather than let it wrap uint32
    // (4.1 days) or lose integer exactness in the float conversion (23 min).
    // Capping only freezes reported time; the angle integral is unaffected.
    if (!st.done && s_sp[m].ticks < TICKS_MAX) ++s_sp[m].ticks;
    s_sp[m].prog = st;
    // Integrate the COMMANDED angle instead of driving it from st.pos: at
    // 25000 rad a float resolves only ~2 mrad, which is 6 electrical degrees
    // of quantisation noise on the field. Wrapping at one mechanical
    // revolution keeps the integral small and is exact, because POLE_PAIRS is
    // an integer and so the electrical angle is untouched by the wrap.
    float th = theta + vel * CTRL_DT;
    while (th >= TWO_PI_F) th -= TWO_PI_F;
    while (th < 0.0f) th += TWO_PI_F;
    s_sp[m].theta_mech = th;  // for the NEXT tick
  }
  spin_unlock(s_lock, irq);

  if (!enabled || mode == MODE_OFF) {
    piReset(s_pid[m]); piReset(s_piq[m]);
    s_lq[m] = PHASE_LQ;
    s_next_a[m] = 0.0f; s_next_b[m] = 0.0f;
    return;
  }

  if (mode == MODE_AB) {
    // Direct stationary-frame duty mode: bypass all transforms and the PI.
    // Used by phases 1c and 5 to drive a known voltage without dq-frame
    // complexity in the path.
    piReset(s_pid[m]); piReset(s_piq[m]);
    auto clamp1 = [](float x){ return x < -1.0f ? -1.0f : (x > 1.0f ? 1.0f : x); };
    s_next_a[m] = clamp1(va_duty);
    s_next_b[m] = clamp1(vb_duty);
    return;
  }

  const float dt = CTRL_DT; // 12 kHz per motor
  float theta_e = electricalAngle(theta);
  // Electrical speed is exact in profile mode -- it is the commanded
  // velocity, so no estimator is needed. Voltage mode commands no motion of
  // its own, so it has none.
  float we = (mode == MODE_PROFILE) ? vel * (float)POLE_PAIRS : 0.0f;

  float ud, uq;
  DQ i_dq = {0.0f, 0.0f};

  if (mode == MODE_VOLTAGE) {
    // Open-loop: uq is commanded directly in volts, ud=0, PI not touched.
    ud = 0.0f;
    uq = uq_cmd;
  } else {
    DQ dq = park(i, theta_e);
    i_dq = dq;
    // Voltage-limited current derate, FEEDFORWARD from speed.
    // A feedback version that measures |ud| and scales by (ud_lim/|ud|)
    // cannot work, because piStep already clamps its output to +-vbus: |ud|
    // therefore never exceeds the bus, the ratio never drops below UD_FRAC,
    // and the loop cannot cut more than 15% no matter how far over budget it
    // really is. Computing the limit from speed has no such blind spot:
    //     ud = -we*Lq*iq   =>   iq_max = UD_FRAC*vbus / (|we|*Lq)
    // It is instant, cannot saturate, and needs no loop dynamics.
    float we_mag = fabsf(we);
    float iq_eff = MOTOR_AMPS;
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

    // Separate KP per axis: this motor is salient (Lq = 2.2 * Ld), so one
    // gain cannot place both poles. KI is shared -- see constants.h.
    uq = piStep(s_piq[m], iq_eff - dq.q, KP_Q, KI, dt, vbus);
    ud = piStep(s_pid[m], 0.0f   - dq.d, KP_D, KI, dt, vbus);

    // Cross-coupling decoupling, driven by the COMMAND, not the measurement.
    //     Ld*did/dt = ud - R*id + we*Lq*iq
    //     Lq*diq/dt = uq - R*iq - we*Ld*id - we*lm
    // Cancelling the off-diagonal terms needs -we*Lq*iq on d and +we*Ld*id
    // on q. Using the MEASURED currents for that is a trap: it closes a loop
    // ud -> id -> uq -> iq -> ud whose gain is (we*Lq)(we*Ld)/(Zd*Zq), and at
    // high speed Zd -> we*Ld and Zq -> we*Lq, so the gain approaches exactly
    // 1 -- marginally stable before the pipeline delay is even counted. It
    // also amplifies current-sense ripple by we*Lq, which is 30 V/A at
    // 690 RPM. Measured on hardware the ceiling dropped from 690 to 550 RPM.
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
    // and uq TOGETHER so the applied voltage vector keeps its angle: zeroing
    // uq outright removes all torque at exactly the speed where torque is
    // needed, and turns the ceiling into a cliff.
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
  s_next_a[m] = v.a;
  s_next_b[m] = v.b;

  // Publish the whole dq snapshot in one locked write, so a caller polling
  // the four getters cannot pair ud from one tick with iq from the next.
  uint32_t irq2 = spin_lock_blocking(s_lock);
  s_telu[m].d = ud;      s_telu[m].q = uq;
  s_teli[m].d = i_dq.d;  s_teli[m].q = i_dq.q;
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

void focEnable(Motor m, bool enable) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].enabled = enable;
  // Disabling rewinds the profile clock instead of freezing it. There is no
  // position sensor, so the commanded angle IS the assumed rotor angle: the
  // bridge is about to be de-energised and the rotor will coast to rest, and
  // resuming mid-cruise would then command 1400 Hz electrical at a stationary
  // rotor, which cannot pull in. It would buzz while motorProgress() reported
  // the move completing normally. Re-enabling replays the move from rest.
  // prog is deliberately left reporting where the axis actually stopped --
  // rewinding it too would make a finished-then-disabled axis read back as
  // position 0, which is what the bringup traces print.
  if (!enable) s_sp[m].ticks = 0;
  spin_unlock(s_lock, irq);
}

// theta_mech is deliberately left alone: the profile is relative, so the move
// starts from wherever the axis is now.
void focSetProfile(Motor m, const Profile& p) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].mode = MODE_PROFILE;
  s_sp[m].prof = p;
  s_sp[m].cruise = false;
  s_sp[m].ticks = 0;
  s_sp[m].prog = p.at(0.0f);
  spin_unlock(s_lock, irq);
}

void focSetVelocity(Motor m, float vel_rad_s) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].mode = MODE_PROFILE;
  s_sp[m].prof = Profile();
  s_sp[m].cruise = true;
  s_sp[m].cruise_vel = vel_rad_s;
  s_sp[m].ticks = 0;
  s_sp[m].prog = ProfileState{0.0f, 0.0f, vel_rad_s, 0.0f, false};
  spin_unlock(s_lock, irq);
}

void focSetVoltage(Motor m, float theta_mech, float uq_volts) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].mode = MODE_VOLTAGE;
  s_sp[m].theta_mech = theta_mech;
  s_sp[m].uq = uq_volts;
  spin_unlock(s_lock, irq);
}

void focSetVoltageAB(Motor m, float va_duty, float vb_duty) {
  uint32_t irq = spin_lock_blocking(s_lock);
  s_sp[m].mode = MODE_AB;
  s_sp[m].va_duty = va_duty;
  s_sp[m].vb_duty = vb_duty;
  spin_unlock(s_lock, irq);
}

Profile focProfile(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  Profile p = s_sp[m].prof;
  spin_unlock(s_lock, irq);
  return p;
}

ProfileState focProgress(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  ProfileState st = s_sp[m].prog;
  spin_unlock(s_lock, irq);
  return st;
}

AB focTelemetry(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  AB t; t.a = s_tel[m].a; t.b = s_tel[m].b;
  spin_unlock(s_lock, irq);
  return t;
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

} // namespace rotev
