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

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)

  TrigonometryState(states_msgs::msg::StatesServos* f_servos, states_msgs::msg::StatesServos* b_servos,
    rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* f_pub,
    rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* b_pub,
    float _l1, float _l2, float _leg_width, std::array<std::array<float, 2>, 12> _data) 
    : front_servos(f_servos), back_servos(b_servos),
    front_state_publisher(f_pub), back_state_publisher(b_pub),
    l1(_l1), l2(_l2), leg_width(_leg_width), data(_data) { }

  float convert_rad_to_deg(float rad);
  void move_servo_to_ang(MicroController mc, int servo, float angle);
  Vector3 set_leg_to(Vector3 xyz);
  void set_legs_distance_to(float value);

  float l1 = 14, l2 = 14, leg_width = 2.5f;
  std::array<std::array<float, 2>, 12> data;
};

} // namespace smov
