#pragma once

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class ManualWakeUpState {
 public:
  STATE_CLASS("Manual Wake Up")

  int cooldown = 2;
};

} // namespace smov
