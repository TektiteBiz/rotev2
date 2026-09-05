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
// Published inside the same locked write as s_telu/s_teli, so a caller polling
// several getters cannot pair trim from one tick with ud from the next. The
// s_trim/s_iqcmd working copies are ISR-private and must not be read directly.
// NOTE: each getter still takes the lock separately, so this gives per-value
// coherence, NOT cross-getter atomicity -- a caller reading trim then ud can
// still straddle two ticks. Use focTelemetryI/U for the pairs that must match.
static volatile float s_teltrim[2] = {1.0f, 1.0f};
static volatile float s_teliqc[2]  = {MOTOR_HOLD_AMPS, MOTOR_HOLD_AMPS};
static PIState s_pid[2], s_piq[2];
static float   s_lq[2] = {PHASE_LQ, PHASE_LQ};  // online Lq, tracks saturation
// Saturation trim on the derate, plus its two input filters. See constants.h.
static float   s_trim[2]  = {1.0f, 1.0f};
static float   s_sat[2]   = {0.0f, 0.0f};   // backstop duty cycle
// Slewed q-axis current command, so the move/hold transition is not a step.
static float   s_iqcmd[2] = {MOTOR_HOLD_AMPS, MOTOR_HOLD_AMPS};
static float   s_donefor[2] = {0.0f, 0.0f};   // seconds the axis has been done
// Previous tick's backstop flag. Gates the s_lq estimator: the FILTERED duty
// was far too blunt -- s_sat < 0.02 means fewer than 2% of ticks clipped, which
// on the validation run shut the estimator out of 90% of windows above 400 RPM,
// exactly where s_lq is the only thing making the derate correct.
static bool    s_satprev[2] = {false, false};
static float   s_vbad[2] = {0.0f, 0.0f};  // seconds of invalid vbus
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
  // Guard: adcExtVbus() is 0.0 until the first ADS1015 sequence lands, and an
  // I2C fault can make it negative. A negative vbus inverts every clampf() in
  // piStep/inversePark and makes the backstop scale factor negative. The !(>)
  // form also rejects NaN.
  float vbus = adcExtVbus();
  if (!(vbus > 1.0f) || vbus > 2.0f * VBUS_V) {
    // Substituting VBUS_V indefinitely would drive blind on an ASSUMED rail --
    // worse than not driving, because duties are u/VBUS_V and a real rail above
    // that lands proportionally more current. adcExtVbus() reads 0.0 until the
    // first ADS1015 sequence completes and has no validity flag, so allow a
    // short startup window and then refuse. The upper clamp catches an I2C
    // glitch reading absurdly high, which would otherwise wind both PI
    // integrators to that value through piStep's out_limit.
    if (s_vbad[m] < VBUS_FAULT_S) {
      s_vbad[m] += CTRL_DT;
      vbus = VBUS_V;
    } else {
      s_next_a[m] = 0.0f; s_next_b[m] = 0.0f;
      piReset(s_pid[m]); piReset(s_piq[m]);
      return;
    }
  } else {
    s_vbad[m] = 0.0f;
  }

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
  bool  holding = false;
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
    // A finished move holds; so does a cruise commanded to zero velocity.
    // Gate on st.done rather than on vel, so a move that passes through zero
    // velocity mid-sequence does not drop current at the worst moment.
    if (s_sp[m].cruise) {
      holding = fabsf(s_sp[m].cruise_vel) < 1e-3f;
      s_donefor[m] = 0.0f;
    } else {
      s_donefor[m] = st.done ? (s_donefor[m] + CTRL_DT) : 0.0f;
      holding = s_donefor[m] >= HOLD_DWELL_S;
    }
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
    // NOTE: PHASE_LQ was characterised at 0.5 A (constants.h) while the drive
    // runs at MOTOR_AMPS, so this seed is conservative (over-derates) for the
    // ~83 ms the estimator takes to converge. Left as-is deliberately: erring
    // high is the safe direction, and correcting it depends on a separation of
    // Ld from the magnet flux that has not been measured yet.
    s_lq[m] = PHASE_LQ;
    s_trim[m] = 1.0f; s_sat[m] = 0.0f;
    s_donefor[m] = 0.0f; s_satprev[m] = false;
    s_teltrim[m] = 1.0f; s_teliqc[m] = MOTOR_HOLD_AMPS;
    // Hold, not MOTOR_AMPS: focEnable() promotes an axis with no profile to a
    // standstill hold, so starting at MOTOR_AMPS energises at 0.9 A and decays
    // -- a 2.25x torque snap at exactly the moment callers try to make gentle.
    s_iqcmd[m] = MOTOR_HOLD_AMPS;
    s_next_a[m] = 0.0f; s_next_b[m] = 0.0f;
    return;
  }

  if (mode == MODE_AB) {
    // Direct stationary-frame duty mode: bypass all transforms and the PI.
    // Used by phases 1c and 5 to drive a known voltage without dq-frame
    // complexity in the path.
    piReset(s_pid[m]); piReset(s_piq[m]);
    // Otherwise a saturating profile leaves s_trim at 0.70, and the next
    // profile resumes with a 30% cut for the ~300 ms the up-rate needs.
    // Published too: these paths return before the normal telemetry write, so
    // without this the getters report the last profile's values indefinitely.
    s_trim[m] = 1.0f; s_sat[m] = 0.0f; s_iqcmd[m] = MOTOR_HOLD_AMPS;
    s_teltrim[m] = 1.0f; s_teliqc[m] = MOTOR_HOLD_AMPS;
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
    s_trim[m] = 1.0f; s_sat[m] = 0.0f; s_iqcmd[m] = MOTOR_HOLD_AMPS;
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
    // Full current while moving, reduced while holding. Slewed so the
    // transition is not a torque step -- see MOTOR_HOLD_AMPS in constants.h.
    float iq_want = holding ? MOTOR_HOLD_AMPS : MOTOR_AMPS;
    float dcmd = iq_want - s_iqcmd[m];
    float up   = HOLD_SLEW_UP_A_PER_S   * dt;
    float down = HOLD_SLEW_DOWN_A_PER_S * dt;
    if (dcmd >  up)   dcmd =  up;
    if (dcmd < -down) dcmd = -down;
    s_iqcmd[m] += dcmd;

    float we_mag = fabsf(we);
    float iq_eff = s_iqcmd[m];
    if (we_mag > 1.0f) {
      // s_lq, not PHASE_LQ: Lq falls with current, so a constant measured at
      // 0.5 A under-derates at the low currents the limiter itself produces.
      // Measured on hardware: targeting 0.85 of the bus actually delivered
      // 0.995 and sat pinned at the backstop.
      // Feedforward from speed (fast, tracks the ramp with no lag because we
      // is the COMMANDED velocity), scaled by the trim (slow, corrects what
      // the model cannot see). A pure trim could not do this alone: a 0.25 s
      // ramp to 600 RPM needs iq_max tracked at ~5 A/s, which a plain
      // integrator only manages with a ~10 ms loop -- fast enough to fight the
      // 83 ms s_lq filter upstream of it.
      float iq_max = (UD_FRAC * vbus) / (we_mag * s_lq[m]) * s_trim[m];
      if (iq_max < IQ_MIN) iq_max = IQ_MIN;
      if (iq_eff >  iq_max) iq_eff =  iq_max;
      if (iq_eff < -iq_max) iq_eff = -iq_max;
    }

    // Separate KP per axis: this motor is salient (Lq = 2.2 * Ld), so one
    // gain cannot place both poles. KI is shared -- see constants.h.
    uq = piStep(s_piq[m], iq_eff - dq.q, KP_Q, KI, dt, vbus);
    ud = piStep(s_pid[m], 0.0f   - dq.d, KP_D, KI, dt, vbus);
    const float ud_pi = ud;  // pre-feedforward, for the estimator gate below

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
    // Skip while the d-axis PI is at its own +-vbus clamp: ud is then the
    // clamp value, not the voltage the loop demanded, so lq_obs reads low and
    // would drag the estimate down exactly when the drive is already short.
    // Reject exactly the ticks whose ud was clipped, using the PREVIOUS tick's
    // backstop flag -- not the filtered duty, which rejects whole regions. The
    // ud_pi term is kept as a guard on the d-axis PI hitting its own clamp;
    // with DECOUPLE_FRAC = 1 the feedforward carries the speed term so ud_pi is
    // normally small, and this only fires if the model is badly wrong.
    if (we_mag > 1.0f && fabsf(dq.q) > LQ_EST_MIN_IQ &&
        fabsf(ud_pi) < 0.98f * vbus && !s_satprev[m]) {
      float lq_obs = fabsf(ud) / (we_mag * fabsf(dq.q));
      if (lq_obs > PHASE_LD && lq_obs < 4.0f * PHASE_LQ)
        s_lq[m] += LQ_ALPHA * (lq_obs - s_lq[m]);
    }

    // Backstop for whatever the derate has not caught up with yet. Scale ud
    // and uq TOGETHER so the applied voltage vector keeps its angle: zeroing
    // uq outright removes all torque at exactly the speed where torque is
    // needed, and turns the ceiling into a cliff.
    bool saturated = false;
    float umag = sqrtf(ud * ud + uq * uq);
    if (umag > vbus) {
      float sc = vbus / umag;
      saturated = true;
      ud *= sc;
      uq *= sc;
      // piStep's anti-windup only sees its own per-axis +-vbus limit, and
      // this scaling happens afterwards -- so without pulling the
      // integrators down by the same factor they wind up against a ceiling
      // they cannot observe. Measured pinned at |u| = vbus continuously.
      s_pid[m].integ *= sc;
      s_piq[m].integ *= sc;
    }

    // --- Saturation trim ---
    // Two independent saturation signals, both already computed: the backstop
    // firing (unambiguous -- the loop demanded more voltage than exists) and
    // loss of d-axis regulation. Filtered so a single-tick transient cannot
    // move the trim. The gap between trip and clear is what stops a limit
    // cycle, which is why recovery can be fast without hunting.
    s_sat[m] += (dt / SAT_FILT_TAU_S) * ((saturated ? 1.0f : 0.0f) - s_sat[m]);

    // Trigger on the BACKSTOP ONLY, not on |id|.
    //
    // |id| is ambiguous: it grows both when the bus runs out AND when the
    // rotor loses sync, because a desynchronised rotor makes park() transform
    // about the wrong angle. Cutting current is the right answer to the first
    // and exactly the wrong answer to the second -- a mechanical overload
    // needs MORE torque, not less. Measured on hardware: hand-loading an axis
    // to stall drove |id| to 0.22 A and pinned this trim at its floor for
    // 1.25 s, deepening the stall it was reacting to.
    //
    // The backstop has no such ambiguity. A stalled rotor generates no
    // back-EMF, so its voltage demand is only R*I -- the backstop cannot fire
    // on a stall, and fires only on genuine voltage saturation.
    //
    if (s_sat[m] > SAT_TRIP) {
      s_trim[m] -= TRIM_DOWN_RATE * dt *
                   ((s_sat[m] - SAT_TRIP) / (1.0f - SAT_TRIP));
    } else if (s_sat[m] < SAT_CLEAR) {
      s_trim[m] += TRIM_UP_RATE * dt;
    }
    if (s_trim[m] < TRIM_MIN) s_trim[m] = TRIM_MIN;
    if (s_trim[m] > 1.0f)     s_trim[m] = 1.0f;
    s_satprev[m] = saturated;
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
  s_teltrim[m] = s_trim[m];  s_teliqc[m] = s_iqcmd[m];
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
  // An axis that has never been given a profile is still MODE_OFF, which the
  // ISR treats as "zero both duties" -- enabling it would release nSLEEP and
  // then leave the shaft free to be turned by hand. Promote it to a standstill
  // hold instead: an empty profile evaluates to zero velocity for all t, so
  // theta_mech never advances and the axis energises at its current commanded
  // angle. That is the same path the ISR already takes once a move finishes,
  // not a new mode. The first focSetProfile()/focSetVelocity()/focSetVoltage*()
  // overwrites this, so a caller that enables before commanding is unaffected.
  if (enable && s_sp[m].mode == MODE_OFF) {
    s_sp[m].mode   = MODE_PROFILE;
    s_sp[m].prof   = Profile();
    s_sp[m].cruise = false;
    s_sp[m].ticks  = 0;
    s_sp[m].prog   = ProfileState{0.0f, 0.0f, 0.0f, 0.0f, true};
  }
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

float focDerateTrim(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  float v = s_teltrim[m];
  spin_unlock(s_lock, irq);
  return v;
}

float focCurrentCmd(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  float v = s_teliqc[m];
  spin_unlock(s_lock, irq);
  return v;
}

DQ focTelemetryI(Motor m) {
  uint32_t irq = spin_lock_blocking(s_lock);
  DQ t; t.d = s_teli[m].d; t.q = s_teli[m].q;
  spin_unlock(s_lock, irq);
  return t;
}

} // namespace rotev
