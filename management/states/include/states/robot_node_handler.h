#pragma once

#include <states/robot_manager.h>

#include "front_board_msgs/srv/servos_config.hpp"
#include "front_board_msgs/msg/servo_config.hpp"
#include "back_board_msgs/srv/servos_config.hpp"
#include "back_board_msgs/msg/servo_config.hpp"
#include "states_msgs/msg/states_servos.hpp"

using namespace std::chrono_literals;

namespace smov {

class RobotNodeHandle : public rclcpp::Node {
 public:
  RobotNodeHandle();

 private:
  void declare_parameters();
  void set_up_publishers();
  void config_servos();
  void call();
  void front_topic_callback(states_msgs::msg::StatesServos::SharedPtr msg);
  void back_topic_callback(states_msgs::msg::StatesServos::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer;

  // Creating the base robot with all the necessary data & publishers.
  RobotManager* robot = RobotManager::Instance();
  
  
  rclcpp::Subscription<states_msgs::msg::StatesServos>::SharedPtr front_states_sub;
  rclcpp::Subscription<states_msgs::msg::StatesServos>::SharedPtr back_states_sub;
  rclcpp::Client<front_board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_pub;
  rclcpp::Client<back_board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_pub;
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_prop_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_prop_pub;
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;
};      

} // namespace smov
