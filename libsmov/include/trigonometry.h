#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include "smov_msgs/msg/states_servos.hpp"

#include "library.h"
#include "mathematics.h"

namespace smov {

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)

  TrigonometryState(smov_msgs::msg::StatesServos *f_servos, smov_msgs::msg::StatesServos *b_servos,
                    rclcpp::Publisher<smov_msgs::msg::StatesServos>::SharedPtr *f_pub,
                    rclcpp::Publisher<smov_msgs::msg::StatesServos>::SharedPtr *b_pub,
                    double *_upper_leg_length, double *_lower_leg_length, double *_hip_body_distance)
      : front_servos(f_servos), back_servos(b_servos),
        front_state_publisher(f_pub), back_state_publisher(b_pub),
        upper_leg_length(_upper_leg_length), lower_leg_length(_lower_leg_length), hip_body_distance(_hip_body_distance) {}

  static float convert_rad_to_deg(float rad);
  void set_leg_to(int leg_group_number, Vector3 xyz);
  void set_legs_distance_to(float value);

  double *upper_leg_length, *lower_leg_length, *hip_body_distance;
};

} // namespace smov
