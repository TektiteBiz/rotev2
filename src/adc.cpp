#include "adc.h"
#include "hardware/adc.h"

namespace rotev {

void adcInit() {
  adc_init();
  adc_gpio_init(PIN_SOB_1); adc_gpio_init(PIN_SOA_1);
  adc_gpio_init(PIN_SOB_2); adc_gpio_init(PIN_SOA_2);
}

static uint16_t oversample(uint ch) {
  adc_select_input(ch);
  uint32_t acc = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; ++i) acc += adc_read();
  return (uint16_t)(acc / ADC_OVERSAMPLE);
}

AB adcSampleMotor(Motor m) {
  uint chA = (m == MOTOR_1) ? ADC_SOA_1 : ADC_SOA_2;
  uint chB = (m == MOTOR_1) ? ADC_SOB_1 : ADC_SOB_2;
  uint16_t a = oversample(chA);
  uint16_t b = oversample(chB);
  return { countsToAmps(a) * ISENSE_SCALE_A + ISENSE_OFFSET_A,
           countsToAmps(b) * ISENSE_SCALE_B + ISENSE_OFFSET_B };
}

} // namespace rotev
