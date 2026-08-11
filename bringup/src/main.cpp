#include <Arduino.h>
#if PHASE == 1
  #include "../phase1_hw.h"
#elif PHASE == 11  // phase1b
  #include "../phase1b_motor.h"
#elif PHASE == 12  // phase1c: stationary-frame PI (no dq transforms)
  #include "../phase1c_stationary_pi.h"
#elif PHASE == 2
  #include "../phase2_openloop.h"
#elif PHASE == 3
  #include "../phase3_foc.h"
#elif PHASE == 4
  #include "../phase4_full.h"
#elif PHASE == 5  // R/L characterization
  #include "../phase5_rl.h"
#endif
