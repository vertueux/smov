#pragma once

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)
  TrigonometryState(float _l1, float _l2, float _l3);

  float l1;
  float l2;
  float l3;

  void solve_leg_kinematics(float x, float y, float z, bool is_right);
};

} // namespace smov
