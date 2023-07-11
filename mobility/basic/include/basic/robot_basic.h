#pragma once

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class BasicState {
 public:
  STATE_CLASS("Basic")
  
  // Used for proportional servos.
  void set_servos_to(float value); 
  void set_servos_to_center(); 

  // Both are used for absolute servos.
  void set_servos_to_min(); 
  void set_servos_to_max(); 
};

} // namespace smov
