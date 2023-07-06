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

class RobotStates {
 public: 
  static RobotStates *Instance();

  void on_start();

  // Arrays to publish in the proportional publisher.
  front_board_msgs::msg::ServoArray front_prop_array;
  back_board_msgs::msg::ServoArray back_prop_array;
  
  // Arrays to publish in the absolute publisher.
  front_board_msgs::msg::ServoArray front_abs_array;
  back_board_msgs::msg::ServoArray back_abs_array;

  // Setting up the servos to their corresponding port.
  void set_up_abs_servos();

  // We push all the front servos into the absolute arrays to be published.
  void update_abs_array();

  // We push all the front servos into the proportional arrays to be published.
  void update_prop_array();

  // Proportional servos.
  static FrontServoArray front_prop_servos;
  static BackServoArray back_prop_servos;

  // Absolute servos.
  static FrontServoArray front_abs_servos;
  static BackServoArray back_abs_servos;

  static std::vector<std::vector<long int>> front_servos_data;
  static std::vector<std::vector<long int>> back_servos_data;

  static double pulse_for_angle;

  std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVBG", 
                                            "AVBD", "AVJG", "AVJD",
                                            "ARBG", "ARBD", "ARCG",
                                            "ARCD", "ARJG", "ARJD"};

 private:
  RobotStates& operator= (const RobotStates&) = delete;
  RobotStates (const RobotStates&) = delete;
  static RobotStates *instance;
  RobotStates();
  ~RobotStates();
};

} // namespace smov
