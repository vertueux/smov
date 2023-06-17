#pragma once

#include <states/smov_states.h>

namespace smov {

class Behaviors {
 public:
  static void lock_servos(FrontServoArray f_servos, BackServoArray b_servos); 
};

} // namespace smov
