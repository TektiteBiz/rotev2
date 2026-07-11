#pragma once
#include "constants.h"
#include "foc_math.h"   // AB
namespace rotev {
void adcInit();
void adcCalibrate();   // call once at startup with motors disabled (measures zero-current offset)
AB   adcSampleMotor(Motor m);
}
