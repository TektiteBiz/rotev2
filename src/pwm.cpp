#include "pwm.h"
#include <initializer_list>
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico.h"

namespace rotev {

static uint16_t s_top = 0;


// Initializes the physical slice shared by pin's channel and its aliased
// partner. Call exactly once per unique slice -- pwm_init() resets BOTH
// channels' compare registers, so a second call on an already-configured
// slice would clobber the first pin's duty (see docs/PRD.md Known Pitfalls).
static void cfgSlice(uint32_t pin) {
  unsigned slice = pwm_gpio_to_slice_num(pin);
  pwm_config c = pwm_get_default_config();
  pwm_config_set_phase_correct(&c, true);
  pwm_config_set_wrap(&c, s_top);
  pwm_config_set_clkdiv(&c, 1.0f);
  pwm_init(slice, &c, false);   // don't start yet; start all slices together below
}

void pwmInit() {
  // phase-correct: f = f_sys / (2 * (TOP+1)) -> TOP = f_sys/(2*PWM_HZ) - 1
  s_top = (uint16_t)(clock_get_hz(clk_sys) / (2u * PWM_HZ) - 1u);
  // PHA_2/PHB_2 share slice 0 (channels A/B), PHA_1/PHB_1 share slice 1.
  // Locked-antiphase: EN is hardwired HIGH on the PCB (no GPIO), PH is
  // PWMed at duty=(1+d)/2 for all 4 phases uniformly.
  cfgSlice(PIN_PHA_2);
  cfgSlice(PIN_PHA_1);
  for (uint32_t pin : {PIN_PHA_2, PIN_PHB_2, PIN_PHA_1, PIN_PHB_1}) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_gpio_level(pin, s_top / 2);  // 50% duty = 0 current
  }
  // Start both slices simultaneously from counter=0 so their PWM edges are
  // in a known, fixed relationship with the master-slice IRQ.
  // pwm_set_mask_enabled() writes the WHOLE enable register, not just the
  // given bits -- it would silently disable every other already-running
  // slice (e.g. the LED's, enabled earlier in ledInit()). OR the mask in
  // instead so unrelated slices are left alone.
  uint32_t mask = (1u << pwm_gpio_to_slice_num(PIN_PHA_2)) |
                  (1u << pwm_gpio_to_slice_num(PIN_PHA_1));
  pwm_hw->en |= mask;
}

unsigned pwmMasterSlice() { return pwm_gpio_to_slice_num(PIN_PHA_2); }

// Locked-antiphase: EN is held HIGH continuously (hardwired on the PCB) and
// PH is PWMed at duty=(1+duty_signed)/2, so current is always actively
// driven through the H-bridge (never coasting) -- the decay behavior FOC
// needs.
void __not_in_flash_func(pwmSetPhase)(uint32_t ph_pin, float duty_signed) {
  float d = duty_signed;
  if (d > 1.0f) d = 1.0f;
  if (d < -1.0f) d = -1.0f;
  float mag = (d + 1.0f) * 0.5f; // [-1,1] -> [0,1]
  pwm_set_gpio_level(ph_pin, (uint16_t)(mag * s_top));
}

} // namespace rotev
