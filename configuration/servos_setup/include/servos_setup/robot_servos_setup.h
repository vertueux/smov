#pragma once

#define SERVO_MAX_SIZE 6

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "board_msgs/srv/servos_config.hpp"
#include "board_msgs/msg/servo_config.hpp"

using namespace std::chrono_literals;

namespace smov {

class ServosSetup : public rclcpp::Node {
 public:
  ServosSetup();

 private:
  void declare_parameters();
  void set_up_topics();
  void config_servos();

  rclcpp::Client<board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_pub;
  rclcpp::Client<board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_pub;

  std::vector<std::vector<long int>> front_servos_data;
  std::vector<std::vector<long int>> back_servos_data;

  std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVBG", 
                                            "AVBD", "AVJG", "AVJD",
                                            "ARCG", "ARCD", "ARBG",
                                            "ARBD", "ARJG", "ARJD"};

  bool use_single_board = false;
};      

} // namespace smov
