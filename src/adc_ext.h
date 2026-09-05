#pragma once
#include "constants.h"
namespace rotev {
void  adcExtInit();          // configures I2C1 + starts the background sampling timer
float adcExtVbus();       // last cached bus voltage (volts), non-blocking
bool  adcExtVbusStale();  // true if the cache has not been refreshed recently
float adcExtUser(AdcChannel ch);  // last cached AIN1/2/3 sample (volts), non-blocking
}
