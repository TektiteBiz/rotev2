#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Closed-loop FOC at constant velocity.
//
// controlStep() has no position sensor -- it takes the COMMANDED theta as a
// stand-in for the true rotor position and uses it to park-transform the
// measured current for feedback. At power-up the real rotor angle is
// arbitrary (nothing has ever aligned it), so that assumption is false from
// sample 0 unless we force it to be true first: hold a fixed theta with a
// modest current for ALIGN_TIME_S so the rotor physically settles into that
// position before the closed loop starts depending on the assumption.
// (Phase 2 doesn't need this -- open-loop voltage control never looks at
// measured current to decide its output, so it can't be fed a bad feedback
// projection the way the closed loop can.)
static constexpr float RPM = 1700.0f;
static constexpr float TARGET_AMPS = 0.8f;
static constexpr float ALIGN_TIME_S = 0.5f;
// Set true to stay in the alignment hold forever. With the rotor stationary
// there is no back-EMF and no di/dt, so the applied q-axis voltage is pure
// Ohm's law: uq = I_actual * (R_phase + R_ds(on) + R_shunt) ~= I_actual*3.68.
// The PI drives *measured* current to TARGET_AMPS, so comparing uq against
// that prediction checks the current-sense gain through the voltage side,
// independently of the sense chain itself. Off by 2x in the sense gain shows
// up as uq off by 2x.
static constexpr bool HOLD_ALIGN = false;
static constexpr float RAMP_TIME_S = 2.0f;

static float elapsed = 0.0f;
static uint32_t last_us = 0;
static uint32_t last_print_us = 0;

// Core0 loop() rate instrumentation. The 12 kHz control ISR can only act on
// the setpoint that loop() hands it, so the setpoint refresh rate -- not the
// ISR rate -- sets how finely the commanded field rotates. Electrical angle
// advanced per iteration is (rpm/60)*360*POLE_PAIRS*dt; once that approaches
// a quarter cycle the command stops being a rotating vector. Worst-case dt
// matters more than the average, so track the max, not just the count.
static uint32_t loop_count = 0;
static uint32_t max_dt_us = 0;

// Bus voltage min/max per print window. A single sample at the 40 Hz print
// rate would alias the ~1 kHz ADS1015 update and miss load-induced sag
// entirely, and sag is the whole point: the FOC derates its torque budget
// against this number (uq_budget = sqrt(vbus^2 - ud^2) in foc.cpp), so a dip
// costs real headroom. Sampling every iteration (~1890 Hz) oversamples the
// ADC instead.
static float vbus_min = 1e9f;
static float vbus_max = -1e9f;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  last_us = last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  uint32_t dt_us = now - last_us;
  float dt = dt_us * 1e-6f;
  last_us = now;
  elapsed += dt;

  loop_count++;
  if (dt_us > max_dt_us) max_dt_us = dt_us;
  // Stop accumulating once the ramp is over. `elapsed` is only read to
  // sequence align->ramp->cruise, and letting a float grow without bound
  // while adding microsecond increments to it eventually loses the
  // increment entirely to rounding.
  if (elapsed > ALIGN_TIME_S + RAMP_TIME_S)
    elapsed = ALIGN_TIME_S + RAMP_TIME_S;

  float vbus = busVoltage();
  if (vbus < vbus_min) vbus_min = vbus;
  if (vbus > vbus_max) vbus_max = vbus;

  float rpm_now = 0.0f;
  if (HOLD_ALIGN || elapsed < ALIGN_TIME_S) {
    motorWrite(0.0f, TARGET_AMPS,
               MOTOR_1);  // hold rotor at theta=0 to align it
  } else {
    float ramp_elapsed = elapsed - ALIGN_TIME_S;
    rpm_now =
        (ramp_elapsed < RAMP_TIME_S) ? RPM * (ramp_elapsed / RAMP_TIME_S) : RPM;
    // Command a velocity, not an angle. The control ISR integrates it on its
    // own 83.3 us tick, so a core0 stall (the USB CDC print below is ~500 us)
    // can no longer inject an angle step into the commanded field. Only the
    // slowly-varying ramp rate comes from here now, where jitter is harmless.
    motorWriteVelocity(2.0f * PI * (rpm_now / 60.0f), TARGET_AMPS, MOTOR_1);
  }

  uint32_t print_dt_us = now - last_print_us;
  if (print_dt_us >= 25000) {
    last_print_us = now;
    // Electrical degrees per commanded-field step. Now set by the control ISR
    // tick (PWM_HZ/2 = 12 kHz), not by loop(), so core0 jitter no longer shows
    // up here. Compare against 90 deg (a quarter cycle).
    float deg_e_step =
        (rpm_now / 60.0f) * 360.0f * (float)POLE_PAIRS / (PWM_HZ / 2.0f);
    Serial.print(">sensA:");
    Serial.print(motorCurrentA(MOTOR_1));
    Serial.print(",sensB:");
    Serial.print(motorCurrentB(MOTOR_1));
    Serial.print(",loopHz:");
    Serial.print(loop_count * 1e6f / print_dt_us, 0);
    Serial.print(",maxDtUs:");
    Serial.print(max_dt_us);
    Serial.print(",degEStep:");
    Serial.print(deg_e_step, 1);
    Serial.print(",ud:");
    Serial.print(motorVoltageD(MOTOR_1), 3);
    Serial.print(",uq:");
    Serial.print(motorVoltageQ(MOTOR_1), 3);
    Serial.print(",id:");
    Serial.print(motorCurrentD(MOTOR_1), 3);
    Serial.print(",iq:");
    Serial.print(motorCurrentQ(MOTOR_1), 3);
    Serial.print(",derate:");
    Serial.print(motorDerate(MOTOR_1), 3);
    Serial.print(",vbusMin:");
    Serial.print(vbus_min, 2);
    Serial.print(",vbusMax:");
    Serial.println(vbus_max, 2);
    loop_count = 0;
    max_dt_us = 0;
    vbus_min = 1e9f;
    vbus_max = -1e9f;
  }

  // No delay: the setpoint is consumed by the 12 kHz-per-motor control ISR,
  // so anything slower than that quantizes the commanded field. At 500 RPM a
  // 528 us loop advanced theta in 79 degree electrical jumps; at the ISR rate
  // it is 12.5 degrees.
}
