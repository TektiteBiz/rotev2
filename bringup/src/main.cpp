#include <Arduino.h>
#if PHASE == 1
  #include "../phase1_hw.h"
#elif PHASE == 11  // phase1b
  #include "../phase1b_motor.h"
#elif PHASE == 2
  #include "../phase2_openloop.h"
#elif PHASE == 3
  #include "../phase3_foc.h"
#elif PHASE == 4
  #include "../phase4_full.h"
#endif
