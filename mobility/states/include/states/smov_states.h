#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "front_board_msgs/msg/servo_array.hpp"
#include "front_board_msgs/msg/servo.hpp"
#include "back_board_msgs/msg/servo_array.hpp"
#include "back_board_msgs/msg/servo.hpp"

namespace smov {

// [front,back]_absolute_servo[PORT]_value. 
// Servo number = [PORT] + 1.
struct {
  // Values for the front board.
  int front_absolute_servo0_value  = 100;
  int front_absolute_servo15_value = 548;
  int front_absolute_servo1_value  = 270;
  int front_absolute_servo14_value = 320;
  int front_absolute_servo2_value  = 287;
  int front_absolute_servo13_value = 355;

  // Values for the back board.
  int back_absolute_servo0_value  = 548;
  int back_absolute_servo15_value = 100;
  int back_absolute_servo1_value  = 320;
  int back_absolute_servo14_value = 270;
  int back_absolute_servo2_value  = 210;
  int back_absolute_servo13_value = 355;
} lockAbsoluteConfiguration;

struct FrontServos {
  front_board_msgs::msg::Servo front_servo0;
  front_board_msgs::msg::Servo front_servo1;
  front_board_msgs::msg::Servo front_servo2;
  front_board_msgs::msg::Servo front_servo13;
  front_board_msgs::msg::Servo front_servo14;
  front_board_msgs::msg::Servo front_servo15;
};

struct BackServos {
  back_board_msgs::msg::Servo back_servo0;
  back_board_msgs::msg::Servo back_servo1;
  back_board_msgs::msg::Servo back_servo2;
  back_board_msgs::msg::Servo back_servo13;
  back_board_msgs::msg::Servo back_servo14;
  back_board_msgs::msg::Servo back_servo15;
};

class States {
 public: 
  static States *Instance();
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_publisher;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_publisher;

  front_board_msgs::msg::ServoArray front_array;
  back_board_msgs::msg::ServoArray back_array;

  static FrontServos configure_front_servos();
  static BackServos configure_back_servos();

  // We push all the front servos into the array to be published.
  void push_all_front_servos_in_array(FrontServos f_servos);

  // We push all the back servos into the array to be published.
  void push_all_back_servos_in_array(BackServos b_servos);
 private:
  States& operator= (const States&) = delete;
  States (const States&) = delete;
  static States *instance;
  States();
  ~States();
};

} // namespace smov
