#pragma once

#include <states/robot_states.h>

namespace smov {

class BasicState : public State {
 public:
  STATE_NAME("Basic")
  
  // Used for proportional servos.
  void set_servos_to(double value); 
  void set_servos_to_center(); 

  // Both are used for absolute servos.
  void set_servos_to_min(); 
  void set_servos_to_max(); 
};

} // namespace smov
