#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#include <smov/mathematics.h>

namespace smov {

enum MicroController {
  FRONT = 0,
  BACK = 1
};

class TrigonometryState {
 public:
  TrigonometryState(float _l1, float _l2, std::array<std::array<float, 2>, 12> _data) 
  : l1(_l1), l2(_l2), data(_data) { }

  float l1 = 14, l2 = 14;
  std::array<std::array<float, 2>, 12> data;

  float convert_rad_to_deg(float rad);

  void move_servo_to_ang(MicroController mc, int servo, float angle,
  states_msgs::msg::StatesServos& f_servos, states_msgs::msg::StatesServos& b_servos);

  Vector3 set_leg_to(Vector3 xyz);

  void set_legs_distance_to(float value,
  states_msgs::msg::StatesServos& f_servos, states_msgs::msg::StatesServos& b_servos);
};

} // namespace smov
