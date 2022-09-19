#pragma once

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "i2cpwm_board_msgs/msg/servo_array.hpp"
#include "i2cpwm_board_msgs/msg/servo.hpp"

namespace smov {

class MotionControl : public rclcpp::Node {
 public:
  MotionControl();

  void initialize_servos();
  int number_of_servos = 12;

 private:
  rclcpp::Publisher<i2cpwm_board_msgs::msg::ServoArray>::SharedPtr publisher;
  i2cpwm_board_msgs::msg::ServoArray servo_array;
};

}