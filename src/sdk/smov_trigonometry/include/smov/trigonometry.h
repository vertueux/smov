#ifndef TRIGONOMETRY_H_
#define TRIGONOMETRY_H_

#include <iostream>
#include <vector>
#include <array>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

#include <smov/library.h>
#include <smov_states_msgs/msg/states_servos.hpp>

#include <smov/mathematics.h>

namespace smov {

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)

  TrigonometryState(smov_states_msgs::msg::StatesServos *f_servos, smov_states_msgs::msg::StatesServos *b_servos,
                    rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr *f_pub,
                    rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr *b_pub,
                    double *_upper_leg_length, double *_lower_leg_length, double *_leg_width, double *_hip_body_distance)
      : front_servos(f_servos), back_servos(b_servos),
        front_state_publisher(f_pub), back_state_publisher(b_pub),
        upper_leg_length(_upper_leg_length), lower_leg_length(_lower_leg_length), leg_width(_leg_width), hip_body_distance(_hip_body_distance) {}

  static float convert_rad_to_deg(float rad);
  void set_leg_to(int leg_group_number, Vector3 xyz);
  void set_legs_distance_to(float value);

  double *upper_leg_length, *lower_leg_length, *leg_width, *hip_body_distance;
};

} // namespace smov

#endif // TRIGONOMETRY_H_
