#pragma once

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)
};

} // namespace smov
