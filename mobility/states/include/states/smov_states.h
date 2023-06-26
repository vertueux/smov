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

template <typename T>
struct ServoArray : std::vector<T> {};

struct FrontServoArray : std::array<front_board_msgs::msg::Servo, SERVO_MAX_SIZE> {};
struct BackServoArray : std::array<back_board_msgs::msg::Servo, SERVO_MAX_SIZE> {};

class States {
 public: 
  static States *Instance();
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_publisher;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_publisher;

  front_board_msgs::msg::ServoArray front_array;
  back_board_msgs::msg::ServoArray back_array;

  // Setting up the servos to their corresponding port.
  void set_up_servos(FrontServoArray f_servos, BackServoArray b_servos);

  // We push all the front servos into the array to be published.
  void push_all_servos_in_array(FrontServoArray f_servos, BackServoArray b_servos);

  static FrontServoArray front_servos;
  static BackServoArray back_servos;
  static std::vector<std::vector<double>> front_servos_data;
  static std::vector<std::vector<double>> back_servos_data;

 private:
  States& operator= (const States&) = delete;
  States (const States&) = delete;
  static States *instance;
  States();
  ~States();
};

} // namespace smov