#pragma once
#include <cstdint>
#include "constants.h"
namespace rotev {
uint16_t pwmTop();
void     pwmInit();
unsigned pwmMasterSlice();
void     pwmSetPhase(uint32_t ph_pin, float duty_signed);
}
