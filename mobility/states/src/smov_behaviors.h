#pragma once

#include <states/smov_states.h>

namespace smov {
  
class Behaviors {
 public:
  void lock_all_front_servos(FrontServos f_servos);
  void lock_all_back_servos(BackServos b_servos);
};

} // namespace smov
