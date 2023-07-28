#pragma once

#define SERVO_MAX_SIZE 6

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "front_board_msgs/msg/servo_array.hpp"
#include "front_board_msgs/msg/servo.hpp"
#include "back_board_msgs/msg/servo_array.hpp"
#include "back_board_msgs/msg/servo.hpp"

namespace smov {

class RobotManager {
 public: 
  static RobotManager *Instance();

  // Arrays to publish in the proportional publisher.
  front_board_msgs::msg::ServoArray front_prop_array;
  back_board_msgs::msg::ServoArray back_prop_array;

  // When using a single board.
  front_board_msgs::msg::ServoArray single_back_array;
  
  // Arrays to publish in the absolute publisher.
  front_board_msgs::msg::ServoArray front_abs_array;
  back_board_msgs::msg::ServoArray back_abs_array;

  // Setting up the servos to their corresponding port.
  void set_up_servos();

  // We push all the servos into the arrays to be published.
  void update_servos_arrays();

  std::vector<std::vector<long int>> front_servos_data;
  std::vector<std::vector<long int>> back_servos_data;

  std::string state = "None";

  double pulse_for_angle = 0;

  std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVBG", 
                                            "AVBD", "AVJG", "AVJD",
                                            "ARCG", "ARCD", "ARBG",
                                            "ARBD", "ARJG", "ARJD"};

 private:
  RobotManager& operator= (const RobotManager&) = delete;
  RobotManager (const RobotManager&) = delete;
  static RobotManager *instance;
  RobotManager();
  ~RobotManager();
};

} // namespace smov
