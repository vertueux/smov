#pragma once

#include <states/robot_manager.h>

#include "std_msgs/msg/string.hpp"
#include "front_board_msgs/srv/servos_config.hpp"
#include "front_board_msgs/msg/servo_config.hpp"
#include "back_board_msgs/srv/servos_config.hpp"
#include "back_board_msgs/msg/servo_config.hpp"
#include "states_msgs/msg/states_servos.hpp"
#include "monitor_msgs/msg/display_text.hpp"

using namespace std::chrono_literals;

namespace smov {

class RobotNodeHandle : public rclcpp::Node {
 public:
  RobotNodeHandle();

  static bool use_single_board;

 private:
  void declare_parameters();
  void set_up_topics();
  void config_servos();
  void late_callback();
  void front_topic_callback(states_msgs::msg::StatesServos::SharedPtr msg);
  void back_topic_callback(states_msgs::msg::StatesServos::SharedPtr msg);
  void state_topic_callback(std_msgs::msg::String::SharedPtr msg);

  // Used for fast operations
  rclcpp::TimerBase::SharedPtr timer;

  // Used for non-necessary fast operations
  rclcpp::TimerBase::SharedPtr late_timer;

  // Used to publish on the LCD panel.
  monitor_msgs::msg::DisplayText up_display;
  monitor_msgs::msg::DisplayText down_display;

  // Creating the base robot with all the necessary data & publishers.
  RobotManager* robot = RobotManager::Instance();
  
  rclcpp::Subscription<states_msgs::msg::StatesServos>::SharedPtr front_states_sub;
  rclcpp::Subscription<states_msgs::msg::StatesServos>::SharedPtr back_states_sub;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr last_current_state_pub;

  rclcpp::Client<front_board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_pub;
  rclcpp::Client<back_board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_pub;
  
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_prop_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_prop_pub;
  
  rclcpp::Publisher<front_board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<back_board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;

  rclcpp::Publisher<monitor_msgs::msg::DisplayText>::SharedPtr monitor_pub;
};      

} // namespace smov
