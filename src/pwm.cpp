#include "pwm.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

namespace rotev {

static uint16_t s_top = 0;

uint16_t pwmTop() { return s_top; }

static void cfgEn(uint32_t pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  unsigned slice = pwm_gpio_to_slice_num(pin);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_phase_correct(&c, true);
  pwm_config_set_wrap(&c, s_top);
  pwm_config_set_clkdiv(&c, 1.0f);
  pwm_init(slice, &c, true);
  pwm_set_gpio_level(pin, 0);
}

void pwmInit() {
  // phase-correct: f = f_sys / (2 * (TOP+1)) -> TOP = f_sys/(2*PWM_HZ) - 1
  s_top = (uint16_t)(clock_get_hz(clk_sys) / (2u * PWM_HZ) - 1u);
  cfgEn(PIN_ENA_1); cfgEn(PIN_ENB_1); cfgEn(PIN_ENA_2); cfgEn(PIN_ENB_2);
}

unsigned pwmMasterSlice() { return pwm_gpio_to_slice_num(PIN_ENA_1); }

void pwmSetPhase(uint32_t en_pin, uint32_t ph_pin, float duty_signed) {
  bool positive = duty_signed >= 0.0f;
  gpio_put(ph_pin, positive ? 1 : 0);
  float mag = positive ? duty_signed : -duty_signed;
  if (mag > 1.0f) mag = 1.0f;
  pwm_set_gpio_level(en_pin, (uint16_t)(mag * s_top));
}

} // namespace rotev
