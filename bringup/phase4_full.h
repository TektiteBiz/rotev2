#pragma once
#include <Arduino.h>
#include <rotev.h>
using namespace rotev;

// Trapezoidal position profile to TARGET_REV rotations with a decel envelope.
//
// Commands VELOCITY, not position. The control ISR integrates the angle on
// its own 83.3 us tick (focSetVelocity), so nothing core0 does can inject an
// angle step into the commanded field. Driving this through motorWrite() put
// the field at the mercy of loop()'s rate: with the old delayMicroseconds(500)
// the setpoint advanced in ~60 electrical degree jumps at 400 RPM, which is
// what made this phase sound terrible while phase 3 was smooth. It also fed a
// staircased angle to the speed estimator, and that estimate now drives the
// voltage derate, the cross-coupling decoupling and the delay compensation.
//
// theta below is core0's OWN integral of the same velocity, used only to
// decide where the profile has got to. It is not the commanded angle, so
// slow drift against the ISR's copy is harmless for "travel N revolutions".
// It is a double because the target is 628 rad, where a float's resolution
// (6e-5) is close enough to one iteration's increment to lose counts.

static constexpr float TARGET_REV = 50.0f;
static constexpr float VMAX = 1400.0f / 60.0f * 2.0f * PI;  // rad/s mechanical
static constexpr float AMAX = VMAX / 0.5f;  // reach VMAX in 0.25 s
static constexpr float AMPS = 0.8f;

static double theta = 0.0;
static float vel = 0.0f;
static uint32_t last_us = 0, last_print_us = 0;

void setup() {
  Serial.begin(115200);
  begin();
  motorEnable(MOTOR_1);
  last_us = last_print_us = micros();
}

void loop() {
  uint32_t now = micros();
  float dt = (now - last_us) * 1e-6f;
  last_us = now;

  const double target = (double)TARGET_REV * 2.0 * PI;
  double remaining = target - theta;
  if (remaining <= 0.0) {
    vel = 0.0f;
  } else {
    float vstop = sqrtf(2.0f * AMAX * (float)remaining);  // decel envelope
    float vcmd = (vel < VMAX) ? vel + AMAX * dt : VMAX;
    if (vcmd > vstop) vcmd = vstop;
    vel = vcmd;
    theta += (double)vel * dt;
  }
  motorWriteVelocity(vel, AMPS, MOTOR_1);

  if (now - last_print_us >= 25000) {
    last_print_us = now;
    Serial.print(">rev:");
    Serial.print((float)(theta / (2.0 * PI)), 2);
    Serial.print(",rpm:");
    Serial.print(vel * 60.0f / (2.0f * PI), 0);
    Serial.print(",sensA:");
    Serial.print(motorCurrentA(MOTOR_1));
    Serial.print(",sensB:");
    Serial.print(motorCurrentB(MOTOR_1));
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
    Serial.print(",vbus:");
    Serial.println(busVoltage(), 2);
  }
  // No delay: the setpoint is consumed by the 12 kHz control ISR.
}
