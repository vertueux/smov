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
  STATE_LIBRARY_CLASS(TrigonometryState)

  TrigonometryState();
  TrigonometryState(states_msgs::msg::StatesServos* f_servos, states_msgs::msg::StatesServos* b_servos,
    rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* f_pub,
    rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* b_pub,
    std::array<std::array<float, 2>, 12> _data, // 0: center angle, 1: max angle.
    float _length_body_shoulder, // L1.
    float _length_shoulder_leg)  // L2.
   :  front_servos(f_servos), back_servos(b_servos), front_state_publisher(f_pub), back_state_publisher(b_pub), 
      l1(_length_body_shoulder), l2(_length_shoulder_leg), data(_data) { }

  float l1 = 14, l2 = 14;
  std::array<std::array<float, 2>, 12> data;

  float convert_rad_to_deg(float rad);
  void move_servo_to_ang(MicroController mc, int servo, float angle);
  Vector3 set_leg_to(Vector3 xyz);
  void set_legs_distance_to(float value);
};

} // namespace smov
