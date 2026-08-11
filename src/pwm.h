#pragma once
#include <cstdint>
#include "constants.h"
namespace rotev {
void     pwmInit();
unsigned pwmMasterSlice();
void     pwmSetPhase(uint32_t ph_pin, float duty_signed);
}
