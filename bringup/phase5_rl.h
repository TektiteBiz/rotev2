#pragma once
#include <Arduino.h>
#include <math.h>
#include <rotev.h>
using namespace rotev;

// Precise winding R and L characterization.
//
// Both values in constants.h started as datasheet numbers and both were
// wrong -- L by 2x -- which halved the current-loop bandwidth and doubled
// the inductive voltage the loop fights at speed. This phase measures them
// from the hardware instead, using open-loop voltage (focSetVoltageAB, no PI
// in the path) and the existing current sense.
//
// WHAT IS MEASURED
//   R  : total series resistance the controller actually sees -- winding +
//        DRV8874 Rds(on) + shunt. That is the correct R for both the PI
//        tuning and for L = tau*R, so no attempt is made to separate them.
//   Ld : inductance on the rotor's magnet axis. The rotor is first parked by
//        a DC current in phase B, which makes B the d-axis; stepping B then
//        produces zero torque, so the rotor cannot move and cannot inject
//        back-EMF into the measurement. This one is trustworthy unattended.
//   Lq : measured on the SPINNING machine, not at standstill. Any q-axis
//        current at standstill makes torque, the rotor moves, and the
//        back-EMF wrecks the fit -- the locked-rotor attempt returned
//        R2 = 0.95 with R_step 19% above the known R, and the shaft could
//        not be held by hand. At steady speed with id driven to 0 the dq
//        model collapses to two clean straight lines:
//              ud = -we*Lq*iq            uq = R*iq + we*lm
//        so sweeping speed and regressing gives Lq from one slope and the
//        magnet flux lm from the other, at the real operating point. Lq is
//        the value the FOC actually uses (it sets ud, the term that eats
//        the voltage budget at speed).
//
// METHOD
// A voltage step on a series RL gives i(t) = i_inf + (i0-i_inf)*exp(-t/tau),
// tau = L/R. Taking the residual (i_inf - i(t)) and fitting ln(residual)
// against t by least squares recovers -1/tau as the slope.
//
// The slope is used rather than any absolute timing because the commanded
// voltage does not reach the winding immediately: core0 writes the setpoint,
// the ISR picks it up on its next pass for this motor (0-83.3 us), and
// applies it on the pass after that (+83.3 us) -- roughly 125 us, and not
// constant. Any such delay shifts ln(residual) vertically, so it lands
// entirely in the intercept and leaves the slope untouched. Verified in
// simulation: L bias stays within +-0.35% for delays from 0 to 400 us.
//
// Two refinements, both needed to reach sub-percent accuracy:
//   - Weighted least squares. var(ln r) ~ (sigma/r)^2, so small residuals are
//     wildly noisy AND biased low (Jensen: E[ln(x+n)] < ln(E[x])). Weighting
//     by r^2 and cutting the window at r > 0.15 drops L bias from +1.9% to
//     +0.4%.
//   - Repeat averaging. REPS captures averaged sample-by-sample before
//     fitting takes the spread from 0.94% to 0.22%.
//
// R comes from a multi-point sweep regressed as I vs V. Using the SLOPE
// rather than any single V/I point makes it immune to current-sense offset
// and to the driver's fixed voltage drop -- both land in the intercept,
// which is reported separately as a diagnostic.
//
// This motor is salient: Ld came out ~3.8 mH against an Lq near 7 mH. That
// is expected for a hybrid stepper -- the magnet sits in the d-axis flux
// path with mu_r ~ 1, so Ld is the LOW one. constants.h carries a single
// PHASE_L for both axes, which cannot be right for both; PHASE_L should
// track Lq, since that is what sets ud.
//
// THE SPEED SWEEP MUST STAY IN SYNC. There is no position sensor, so if the
// rotor pulls out the numbers are meaningless. Speeds below are kept well
// under the observed ~400 RPM ceiling, and mean id is printed per point as
// the check: it should sit near 0. A point with |id| > 0.05 A is flagged.

static constexpr int      N_PRE = 32;    // samples before the step (gives i0)
static constexpr int      N_POST = 480;  // samples after (~29 tau at 7 mH)
static constexpr int      N_TOT = N_PRE + N_POST;
static constexpr uint32_t SAMPLE_US = 100;
static constexpr int      REPS = 16;

// Fit window on the normalised residual. Upper bound skips the pipeline
// delay region, lower bound skips where log-noise bias takes over.
static constexpr float    FIT_LO = 0.15f, FIT_HI = 0.90f;
static constexpr uint32_t FIT_SKIP_US = 400;

static constexpr float V_ALIGN = 3.0f;  // parks the rotor, ~0.70 A
static constexpr float V_LO = 1.0f, V_HI = 3.0f;  // Ld step

// Lq sweep: closed-loop FOC at constant speed, well under the ~400 RPM
// pull-out ceiling so the rotor is guaranteed to stay in sync.
static constexpr float    LQ_AMPS = 0.5f;
static const     float    LQ_RPM[] = {100.0f, 150.0f, 200.0f, 250.0f, 300.0f};
static constexpr int      N_LQ = sizeof(LQ_RPM) / sizeof(LQ_RPM[0]);
static constexpr uint32_t LQ_SETTLE_MS = 400;
static constexpr uint32_t LQ_AVG_MS = 300;
static constexpr float    ID_TOL = 0.05f;

static const float R_SWEEP_V[] = {0.5f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f};
static constexpr int N_SWEEP = sizeof(R_SWEEP_V) / sizeof(R_SWEEP_V[0]);

static float buf[N_TOT];

struct Fit {
  float R, L, tau, r2, i0, iinf;
  int n;
};

static inline float axisCurrent(bool axis_a) {
  return axis_a ? motorCurrentA(MOTOR_1) : motorCurrentB(MOTOR_1);
}

static inline void applyV(float va, float vb) {
  motorWriteVoltageAB(va, vb, MOTOR_1);
}

// Average REPS step responses into buf[], sampled on a fixed 100 us grid.
static void captureStep(bool axis_a, float v_other, float v_from, float v_to) {
  for (int k = 0; k < N_TOT; ++k) buf[k] = 0.0f;

  for (int r = 0; r < REPS; ++r) {
    if (axis_a) applyV(v_from, v_other);
    else        applyV(v_other, v_from);
    delay(40);  // >5 tau of settling at the pre-step level

    uint32_t t0 = micros();
    for (int k = 0; k < N_TOT; ++k) {
      uint32_t target = t0 + (uint32_t)k * SAMPLE_US;
      while ((int32_t)(micros() - target) < 0) {
      }
      if (k == N_PRE) {  // step lands on the sample grid
        if (axis_a) applyV(v_to, v_other);
        else        applyV(v_other, v_to);
      }
      buf[k] += axisCurrent(axis_a);
    }
  }
  for (int k = 0; k < N_TOT; ++k) buf[k] /= (float)REPS;
}

static Fit fitStep(float v_from, float v_to) {
  Fit f = {0, 0, 0, 0, 0, 0, 0};

  double i0 = 0;
  for (int k = 0; k < N_PRE; ++k) i0 += buf[k];
  i0 /= N_PRE;

  int tail0 = N_PRE + (int)(0.8f * N_POST);  // t > 23 tau, settled
  double iinf = 0;
  int nt = 0;
  for (int k = tail0; k < N_TOT; ++k) { iinf += buf[k]; ++nt; }
  iinf /= nt;

  double A = iinf - i0;
  f.i0 = (float)i0;
  f.iinf = (float)iinf;
  if (fabs(A) < 1e-4) return f;  // no step -> nothing to fit

  double sw = 0, swx = 0, swy = 0, swxx = 0, swxy = 0, swyy = 0;
  int n = 0;
  for (int k = N_PRE; k < N_TOT; ++k) {
    double t = (double)(k - N_PRE) * SAMPLE_US * 1e-6;
    if (t < FIT_SKIP_US * 1e-6) continue;
    double r = (iinf - buf[k]) / A;
    if (!(r > FIT_LO && r < FIT_HI)) continue;
    double w = r * r, y = log(r);
    sw += w; swx += w * t; swy += w * y;
    swxx += w * t * t; swxy += w * t * y; swyy += w * y * y;
    ++n;
  }
  if (n < 8) return f;

  double den = sw * swxx - swx * swx;
  double slope = (sw * swxy - swx * swy) / den;
  double ybar = swy / sw;
  double sst = swyy - sw * ybar * ybar;
  double ssr = slope * slope * (swxx - swx * swx / sw);

  f.n = n;
  f.tau = (float)(-1.0 / slope);
  f.R = (float)((v_to - v_from) / A);
  f.L = f.tau * f.R;
  f.r2 = (float)(sst > 0 ? ssr / sst : 0);
  return f;
}

struct OpPoint { float we, ud, uq, id, iq; };

// Ramp the velocity command so the rotor can follow; stepping straight to a
// speed would risk pull-out and silently invalidate everything after it.
static void rampTo(float rpm_from, float rpm_to, uint32_t ms) {
  uint32_t t0 = millis(), el;
  while ((el = millis() - t0) < ms) {
    float rpm = rpm_from + (rpm_to - rpm_from) * ((float)el / (float)ms);
    motorWriteVelocity(2.0f * PI * (rpm / 60.0f), LQ_AMPS, MOTOR_1);
    delay(1);
  }
  motorWriteVelocity(2.0f * PI * (rpm_to / 60.0f), LQ_AMPS, MOTOR_1);
}

static OpPoint measurePoint(float rpm) {
  delay(LQ_SETTLE_MS);
  double aud = 0, auq = 0, aid = 0, aiq = 0;
  uint32_t n = 0, t0 = millis();
  while (millis() - t0 < LQ_AVG_MS) {
    aud += motorVoltageD(MOTOR_1); auq += motorVoltageQ(MOTOR_1);
    aid += motorCurrentD(MOTOR_1); aiq += motorCurrentQ(MOTOR_1);
    ++n;
  }
  OpPoint p;
  p.we = 2.0f * PI * (rpm / 60.0f) * (float)POLE_PAIRS;
  p.ud = (float)(aud / n); p.uq = (float)(auq / n);
  p.id = (float)(aid / n); p.iq = (float)(aiq / n);
  return p;
}

// Unweighted least squares y = a*x + b, plus R^2.
struct Lin { float a, b, r2; };
static Lin linfit(const double* x, const double* y, int n) {
  double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
  for (int k = 0; k < n; ++k) {
    sx += x[k]; sy += y[k]; sxx += x[k] * x[k];
    sxy += x[k] * y[k]; syy += y[k] * y[k];
  }
  double den = n * sxx - sx * sx;
  double a = (n * sxy - sx * sy) / den;
  double b = (sy - a * sx) / n;
  double ybar = sy / n;
  double sst = syy - n * ybar * ybar;
  double ssr = a * a * (sxx - sx * sx / n);
  Lin f; f.a = (float)a; f.b = (float)b;
  f.r2 = (float)(sst > 0 ? ssr / sst : 0);
  return f;
}

static void dumpBuf(const char* tag) {
  Serial.print("# raw ");
  Serial.print(tag);
  Serial.println(" (t_us,amps) -- step at t=0, negative t is pre-step");
  for (int k = 0; k < N_TOT; ++k) {
    Serial.print((int32_t)((int32_t)(k - N_PRE) * (int32_t)SAMPLE_US));
    Serial.print(',');
    Serial.println(buf[k], 5);
  }
}

void setup() {
  Serial.begin(115200);
  begin();
  while (!Serial && millis() < 4000) {
  }
  motorEnable(MOTOR_1);

  Serial.println("# RL characterization");
  Serial.print("# vbus = ");
  Serial.println(busVoltage(), 3);
  Serial.println("# parking rotor on phase B (this becomes the d-axis)...");
  applyV(0.0f, V_ALIGN);
  delay(2000);

  // ---- R: multi-point sweep, regressed as I vs V ----
  Serial.println("# R sweep");
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int i = 0; i < N_SWEEP; ++i) {
    applyV(0.0f, R_SWEEP_V[i]);
    delay(200);
    double acc = 0;
    int n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 100) { acc += motorCurrentB(MOTOR_1); ++n; }
    double I = acc / n, V = R_SWEEP_V[i];
    Serial.print("#   V=");
    Serial.print(V, 3);
    Serial.print("  I=");
    Serial.println(I, 5);
    sx += V; sy += I; sxx += V * V; sxy += V * I;
  }
  double den = N_SWEEP * sxx - sx * sx;
  double slope = (N_SWEEP * sxy - sx * sy) / den;   // A per V
  double icpt = (sy - slope * sx) / N_SWEEP;
  float R_dc = (float)(1.0 / slope);
  float V_off = (float)(-icpt / slope);

  // ---- Ld: rotor parked on B, step B, zero torque ----
  Serial.println("# Ld capture (rotor parked, no torque produced)");
  captureStep(false, 0.0f, V_LO, V_HI);
  Fit ld = fitStep(V_LO, V_HI);
  dumpBuf("Ld");

  // ---- Lq + magnet flux: closed loop, spinning, id held at 0 ----
  Serial.println("# Lq sweep (closed loop, spinning -- keep the shaft free)");
  applyV(0.0f, 0.0f);
  motorWrite(0.0f, LQ_AMPS, MOTOR_1);  // leaves ab_mode, re-aligns at theta=0
  delay(800);
  rampTo(0.0f, LQ_RPM[0], 1500);

  double x_lq[N_LQ], y_lq[N_LQ], x_lm[N_LQ], y_lm[N_LQ];
  bool sync_ok = true;
  for (int k = 0; k < N_LQ; ++k) {
    if (k) rampTo(LQ_RPM[k - 1], LQ_RPM[k], 800);
    OpPoint p = measurePoint(LQ_RPM[k]);
    Serial.print("#   rpm="); Serial.print(LQ_RPM[k], 0);
    Serial.print(" we="); Serial.print(p.we, 1);
    Serial.print(" ud="); Serial.print(p.ud, 4);
    Serial.print(" uq="); Serial.print(p.uq, 4);
    Serial.print(" id="); Serial.print(p.id, 4);
    Serial.print(" iq="); Serial.print(p.iq, 4);
    if (fabsf(p.id) > ID_TOL) { Serial.print("  <-- id OFF ZERO"); sync_ok = false; }
    Serial.println();
    // ud = -we*Lq*iq  ->  slope of ud vs (we*iq) is -Lq
    x_lq[k] = (double)p.we * p.iq;  y_lq[k] = p.ud;
    // uq - R*iq = we*lm  ->  slope vs we is the magnet flux
    x_lm[k] = p.we;                 y_lm[k] = p.uq - (double)R_dc * p.iq;
  }
  Lin f_lq = linfit(x_lq, y_lq, N_LQ);
  Lin f_lm = linfit(x_lm, y_lm, N_LQ);
  float Lq = -f_lq.a;
  float lam = f_lm.a;

  rampTo(LQ_RPM[N_LQ - 1], 0.0f, 1500);
  motorDisable(MOTOR_1);

  Serial.println();
  Serial.println("========= RESULTS =========");
  Serial.print("R (sweep slope)  : "); Serial.print(R_dc, 4); Serial.println(" ohm");
  Serial.print("  driver V offset: "); Serial.print(V_off, 4);
  Serial.println(" V  (dead-time/diode drop; excluded from R by using slope)");
  Serial.print("  sense offset   : "); Serial.print((float)icpt, 5);
  Serial.println(" A  (intercept at V=0; should be ~0)");
  Serial.println();
  Serial.print("Ld  : "); Serial.print(ld.L * 1e3f, 4); Serial.print(" mH   tau=");
  Serial.print(ld.tau * 1e6f, 1); Serial.print(" us  R_step=");
  Serial.print(ld.R, 4); Serial.print(" ohm  R2="); Serial.print(ld.r2, 6);
  Serial.print("  n="); Serial.println(ld.n);
  Serial.print("Lq  : "); Serial.print(Lq * 1e3f, 4); Serial.print(" mH");
  Serial.print("   (slope of ud vs we*iq)  R2="); Serial.print(f_lq.r2, 6);
  Serial.print("  intercept="); Serial.print(f_lq.b, 4);
  Serial.println(" V  (should be ~0)");
  Serial.print("lambda_m: "); Serial.print(lam * 1e3f, 4); Serial.print(" mWb");
  Serial.print("   R2="); Serial.print(f_lm.r2, 6);
  Serial.print("  intercept="); Serial.print(f_lm.b, 4);
  Serial.println(" V  (should be ~0)");
  Serial.print("Kt  : "); Serial.print(lam * (float)POLE_PAIRS, 5);
  Serial.println(" N.m/A   (= POLE_PAIRS * lambda_m; cross-check vs holding torque)");
  Serial.print("saliency Lq/Ld : "); Serial.println(ld.L > 0 ? Lq / ld.L : 0.0f, 3);
  if (!sync_ok) Serial.println("!! a sweep point had id off zero -- likely lost sync, Lq suspect");
  Serial.println();
  Serial.println("# Checks: R_step should agree with the sweep R to ~1%.");
  Serial.println("# Ld R2 below ~0.999 means the fit is not a clean exponential.");
  Serial.println("# Lq/lambda_m R2 below ~0.999, or a nonzero intercept, means the");
  Serial.println("#   speed sweep was not in steady state -- widen LQ_SETTLE_MS.");
  Serial.println("# constants.h wants PHASE_R = sweep R, PHASE_L = Lq.");
}

void loop() {}
