#pragma once

#include <iostream>
#include <vector>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#include <smov/mathematics.h>

namespace smov {

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)

  TrigonometryState();

  float l1 = 10, l2 = 10;

  Vector3 leg_kinematics(Vector3 xyz);
};

} // namespace smov
