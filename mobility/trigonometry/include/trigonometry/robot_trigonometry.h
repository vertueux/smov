#pragma once

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class TrigonometryState {
 public:
  STATE_CLASS_INCLUDE_PARAMS("Trigonometry")

  void set_legs_distance_to(float distance);
};

} // namespace smov
