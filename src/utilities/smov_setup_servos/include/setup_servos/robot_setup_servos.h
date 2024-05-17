#ifndef ROBOT_SERVOS_SETUP_H_
#define ROBOT_SERVOS_SETUP_H_

#define SERVO_MAX_SIZE 6

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "i2c_pwm_board_msgs/srv/servos_config.hpp"
#include "i2c_pwm_board_msgs/msg/servo_config.hpp"

using namespace std::chrono_literals;

namespace smov {

class ServosSetup : public rclcpp::Node {
 public:
  ServosSetup();

 private:
  void declare_parameters();
  void set_up_topics();
  void config_servos();

  rclcpp::Client<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_client;
  rclcpp::Client<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_client;

  std::vector<std::vector<long int>> front_servos_data;
  std::vector<std::vector<long int>> back_servos_data;

  bool use_single_board = false;
  
  std::array<std::string, 12> servo_name = {
    "FRONT_BODY_LEFT",       "FRONT_BODY_RIGHT",     "FRONT_UPPER_LEG_LEFT",
    "FRONT_UPPER_LEG_RIGHT", "FRONT_LOWER_LEG_LEFT", "FRONT_LOWER_LEG_RIGHT",
    "BACK_BODY_LEFT",        "BACK_BODY_RIGHT",      "BACK_UPPER_LEG_LEFT",
    "BACK_UPPER_LEG_RIGHT",  "BACK_LOWER_LEG_LEFT",  "BACK_LOWER_LEG_RIGHT"
  };
};      

} // namespace smov

#endif // ROBOT_SERVOS_SETUP_H_
