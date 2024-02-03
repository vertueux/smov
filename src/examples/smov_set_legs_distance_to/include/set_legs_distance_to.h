#pragma once

#include <errno.h>   // for errno.
#include <limits.h>  // for INT_MAX, INT_MIN.
#include <stdlib.h>  // for strtol.

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#include <smov/trigonometry.h>

namespace smov {

int _argc;
char **_argv;

class LegsDistanceState {
 public:
  STATE_CLASS("Legs Distance Tool")

  std::array<std::array<float, 2>, 12> data = {{{0, 120},{0, 120},{55, 145},{55, 145},{70, 150},{70, 150},  // Front servos.
                                               {0, 120},{0, 120},{55, 145},{55, 145},{70, 150},{70, 150}}}; // Back servos.               
  TrigonometryState trig = TrigonometryState(&front_servos, &back_servos, &front_state_publisher, &back_state_publisher, 14, 14, 2.5f, data);
  int desired_distance = 10;
};

} // namespace smov
