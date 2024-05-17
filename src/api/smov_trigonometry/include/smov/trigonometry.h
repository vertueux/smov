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
                    float _l1, float _l2, float _leg_width, float _shoulder_width)
      : front_servos(f_servos), back_servos(b_servos),
        front_state_publisher(f_pub), back_state_publisher(b_pub),
        l1(_l1), l2(_l2), leg_width(_leg_width), shoulder_width(_shoulder_width) {}

  static float convert_rad_to_deg(float rad);
  void set_leg_to(int leg_number, Vector3 xyz);
  void set_legs_distance_to(float value);

  float l1 = 14, l2 = 14, leg_width = 2.5f, shoulder_width = 4;
};

} // namespace smov

#endif // TRIGONOMETRY_H_
