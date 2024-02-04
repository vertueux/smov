#ifndef ROBOT_MANAGER_H_
#define ROBOT_MANAGER_H_

#define SERVO_MAX_SIZE 6

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "board_msgs/msg/servo_array.hpp"
#include "board_msgs/msg/servo.hpp"

namespace smov {

class RobotManager {
 public:
  static RobotManager *Instance();
  RobotManager(const RobotManager &) = delete;
  RobotManager &operator=(const RobotManager &) = delete;

  // Arrays to publish in the proportional publisher.
  board_msgs::msg::ServoArray front_prop_array;
  board_msgs::msg::ServoArray back_prop_array;

  // When using a single board.
  board_msgs::msg::ServoArray single_back_array;

  // Arrays to publish in the absolute publisher.
  board_msgs::msg::ServoArray front_abs_array;
  board_msgs::msg::ServoArray back_abs_array;

  // Setting up the servos to their corresponding port.
  void set_up_servos();

  // Static until we implement the function
  static void stop_servos();

  // We push all the servos into the arrays to be published.
  // Static until we implement the function
  static void update_servos_arrays();

  std::vector<std::vector<long int>> front_servos_data;
  std::vector<std::vector<long int>> back_servos_data;

  std::string state = "None";

  std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVBG",
                                            "AVBD", "AVJG", "AVJD",
                                            "ARCG", "ARCD", "ARBG",
                                            "ARBD", "ARJG", "ARJD"};

 private:
  static RobotManager *instance;
  RobotManager();
  ~RobotManager();
};

} // namespace smov

#endif // ROBOT_MANAGER_H_