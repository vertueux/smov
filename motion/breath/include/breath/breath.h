#pragma once

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#include <smov/trigonometry.h>

namespace smov {

class BreathState {
 public:
  STATE_CLASS("Breath")

  std::array<std::array<float, 2>, 12> data = {{{0, 70},{0, 70},{45, 135},{45, 135},{-10, 150},{-10, 150},  // Front servos.
                                               {0, 70},{0, 70},{45, 135},{45, 135},{-10, 150},{-10, 150}}}; // Back servos.

  TrigonometryState trig;
};

} // namespace smov
