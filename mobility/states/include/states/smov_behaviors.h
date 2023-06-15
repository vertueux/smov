#pragma once

#include <states/smov_states.h>

namespace smov {
  
class Behaviors {
 public:
  static FrontServos lock_front_phase_one(); 
  static FrontServos lock_front_phase_two(); 
  static FrontServos lock_front_phase_three(); 

  static BackServos lock_back_phase_one(); 
  static BackServos lock_back_phase_two(); 
  static BackServos lock_back_phase_three(); 
  static void wake_up();
};

} // namespace smov
