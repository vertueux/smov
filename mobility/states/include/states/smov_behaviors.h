#pragma once

#include <states/smov_states.h>

namespace smov {
  
class Behaviors {
 public:
  static FrontServos lock_all_front_servos();
  static BackServos lock_all_back_servos();
  static void wake_up();
};

} // namespace smov
