#pragma once

#define SERVO_MAX_SIZE 6

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "front_board_msgs/msg/servo_array.hpp"
#include "front_board_msgs/msg/servo.hpp"
#include "back_board_msgs/msg/servo_array.hpp"
#include "back_board_msgs/msg/servo.hpp"

namespace smov {

struct FrontServoArray : std::array<front_board_msgs::msg::Servo, SERVO_MAX_SIZE> {};
struct BackServoArray : std::array<back_board_msgs::msg::Servo, SERVO_MAX_SIZE> {};

class RobotManager {
 public: 
  static RobotManager *Instance();

  // Called when the node has been created.
  void on_start();

  // Called every 500ms.
  void on_loop();

  // Arrays to publish in the proportional publisher.
  front_board_msgs::msg::ServoArray front_prop_array;
  back_board_msgs::msg::ServoArray back_prop_array;
  
  // Arrays to publish in the absolute publisher.
  front_board_msgs::msg::ServoArray front_abs_array;
  back_board_msgs::msg::ServoArray back_abs_array;

  // Setting up the servos to their corresponding port.
  void set_up_servos();

  // We push all the servos into the arrays to be published.
  void update_servos_arrays();

  // Proportional servos.
  FrontServoArray front_prop_servos;
  BackServoArray back_prop_servos;

  // Absolute servos.
  FrontServoArray front_abs_servos;
  BackServoArray back_abs_servos;

  std::vector<std::vector<long int>> front_servos_data;
  std::vector<std::vector<long int>> back_servos_data;

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
