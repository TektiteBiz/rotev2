#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Closed-loop FOC at constant velocity, on both motors.
//
// controlStep() has no position sensor -- it takes the COMMANDED theta as a
// stand-in for the true rotor position and uses it to park-transform the
// measured current for feedback. At power-up the real rotor angle is
// arbitrary, so that assumption starts out false. The velocity ramp from zero
// is what fixes it: for the first moments the commanded field is barely
// moving while already carrying full current, which is exactly an alignment
// hold, and the rotor is captured long before the speed is high enough to
// outrun it. A separate explicit align stage was redundant with that.
//
// Both axes run the same align -> ramp -> cruise sequence off one clock. The
// 24 kHz ISR alternates motors and so services each at 12 kHz; motor 2 was
// always being stepped, just with a zero setpoint, so enabling it costs
// motor 1 nothing. With nothing plugged into a connector that axis simply
// integrates up against an open circuit -- no current, saturated duty, and
// the online Lq estimator skips itself since it is gated on |iq|. Set an
// axis's `on` field false to leave it idle.
static constexpr float RPM = 1700.0f;
static constexpr float TARGET_AMPS = 0.8f;
static constexpr float RAMP_TIME_S = 2.0f;

struct Axis {
  Motor m;
  bool on;
};
static Axis ax[] = {
    {MOTOR_1, true},
    {MOTOR_2, true},
};
static constexpr int N_AX = sizeof(ax) / sizeof(ax[0]);

static float elapsed = 0.0f;
static uint32_t last_us = 0;
static uint32_t last_print_us = 0;

// Core0 loop() rate instrumentation. Kept even though the ISR now owns the
// commanded angle: it is the check that core0 is still healthy, and
// maxDtUs exposes how long the USB CDC print below actually blocks.
static uint32_t loop_count = 0;
static uint32_t max_dt_us = 0;

// Bus voltage min/max per print window. A single sample at the 40 Hz print
// rate would alias the ~1 kHz ADS1015 update and miss load-induced sag
// entirely, and sag is the whole point: the derate sizes its current limit
// against this number, so a dip costs real headroom. Sampling every
// iteration (~325 kHz) heavily oversamples the ADC instead.
static float vbus_min = 1e9f;
static float vbus_max = -1e9f;

void setup() {
  Serial.begin(115200);
  begin();
  for (int i = 0; i < N_AX; ++i)
    if (ax[i].on) motorEnable(ax[i].m);
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
  // Stop accumulating once the ramp is over. `elapsed` only sequences the
  // ramp, and letting a float grow without bound while adding microsecond
  // increments to it eventually loses the increment entirely to rounding.
  if (elapsed > RAMP_TIME_S) elapsed = RAMP_TIME_S;

  float vbus = busVoltage();
  if (vbus < vbus_min) vbus_min = vbus;
  if (vbus > vbus_max) vbus_max = vbus;

  float rpm_now = RPM * (elapsed / RAMP_TIME_S);
  for (int i = 0; i < N_AX; ++i) {
    if (!ax[i].on) continue;
    // Command a velocity, not an angle. The control ISR integrates it on its
    // own 83.3 us tick, so a core0 stall (the USB CDC print below is ~500 us)
    // cannot inject an angle step into the commanded field. Only the
    // slowly-varying ramp rate comes from here, where jitter is harmless.
    // There is no position target in this phase, so a pure velocity command
    // is correct -- phase 4 needs motorWriteProfile instead, because there an
    // ISR-side integral would drift against the profile's own.
    motorWriteVelocity(2.0f * PI * (rpm_now / 60.0f), TARGET_AMPS, ax[i].m);
  }

  uint32_t print_dt_us = now - last_print_us;
  if (print_dt_us >= 25000) {
    last_print_us = now;
    // Electrical degrees per commanded-field step, set by the control ISR
    // tick (PWM_HZ/2 = 12 kHz). Compare against 90 deg (a quarter cycle);
    // below ~20 steps per electrical cycle the current stops being able to
    // look like a sine no matter how much voltage headroom there is.
    float deg_e_step =
        (rpm_now / 60.0f) * 360.0f * (float)POLE_PAIRS / (PWM_HZ / 2.0f);
    Serial.print(">loopHz:");
    Serial.print(loop_count * 1e6f / print_dt_us, 0);
    Serial.print(",maxDtUs:");
    Serial.print(max_dt_us);
    Serial.print(",rpm:");
    Serial.print(rpm_now, 0);
    Serial.print(",degEStep:");
    Serial.print(deg_e_step, 1);
    Serial.print(",vbusMin:");
    Serial.print(vbus_min, 2);
    Serial.print(",vbusMax:");
    Serial.print(vbus_max, 2);
    for (int i = 0; i < N_AX; ++i) {
      if (!ax[i].on) continue;
      int n = i + 1;
      Serial.print(",sensA");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorCurrentA(ax[i].m));
      Serial.print(",sensB");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorCurrentB(ax[i].m));
      Serial.print(",ud");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorVoltageD(ax[i].m), 3);
      Serial.print(",uq");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorVoltageQ(ax[i].m), 3);
      Serial.print(",id");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorCurrentD(ax[i].m), 3);
      Serial.print(",iq");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorCurrentQ(ax[i].m), 3);
      Serial.print(",derate");
      Serial.print(n);
      Serial.print(':');
      Serial.print(motorDerate(ax[i].m), 3);
    }
    Serial.println();
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
