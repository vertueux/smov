#pragma once

#define SERVO_MAX_SIZE 6

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

struct FrontServoArray : std::array<front_board_msgs::msg::Servo, SERVO_MAX_SIZE> {};
struct BackServoArray : std::array<back_board_msgs::msg::Servo, SERVO_MAX_SIZE> {};

class States {
 public: 
  static States *Instance();

  // Arrays to publish in the proportional publisher.
  front_board_msgs::msg::ServoArray front_prop_array;
  back_board_msgs::msg::ServoArray back_prop_array;
  
  // Arrays to publish in the absolute publisher.
  front_board_msgs::msg::ServoArray front_abs_array;
  back_board_msgs::msg::ServoArray back_abs_array;

  // Setting up the servos to their corresponding port.
  void set_up_servos(FrontServoArray f_servos, BackServoArray b_servos);

  // We push all the front servos into the absolute arrays to be published.
  void push_in_abs_array(FrontServoArray f_servos, BackServoArray b_servos);

  // We push all the front servos into the proportional arrays to be published.
  void push_in_prop_array(FrontServoArray f_servos, BackServoArray b_servos);

  // Proportional servos.
  static FrontServoArray front_prop_servos;
  static BackServoArray back_prop_servos;

  // Absolute servos.
  static FrontServoArray front_abs_servos;
  static BackServoArray back_abs_servos;
  static std::vector<std::vector<long int>> front_servos_data;
  static std::vector<std::vector<long int>> back_servos_data;

  static double pulse_for_angle;

 private:
  States& operator= (const States&) = delete;
  States (const States&) = delete;
  static States *instance;
  States();
  ~States();
};

} // namespace smov
