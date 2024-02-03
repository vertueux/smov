#pragma once

#include <time.h>   
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>

namespace smov {

class ManualWakeUpState {
 public:
  STATE_CLASS("Deprecated Awakening")
  
  int cooldown = 700; // 700 milliseconds.
};

} // namespace smov
