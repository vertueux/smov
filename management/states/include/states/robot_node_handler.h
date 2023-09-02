#pragma once

#include <states/robot_manager.h>

#include "std_msgs/msg/string.hpp"
#include "board_msgs/srv/servos_config.hpp"
#include "board_msgs/msg/servo_config.hpp"
#include "states_msgs/msg/states_servos.hpp"
#include "states_msgs/msg/end_state.hpp"
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
  void end_state_callback(states_msgs::msg::EndState::SharedPtr msg);

  // Used for fast operations
  rclcpp::TimerBase::SharedPtr timer;

  // Used for non-necessary fast operations
  rclcpp::TimerBase::SharedPtr late_timer;

  // Used to publish on the LCD panel.
  monitor_msgs::msg::DisplayText up_display;

  // Used to publish on the LCD panel.
  std::string up_display_str;

  // Creating the base robot with all the necessary data & publishers.
  RobotManager* robot = RobotManager::Instance();
  
  rclcpp::Subscription<states_msgs::msg::StatesServos>::SharedPtr front_states_sub;
  rclcpp::Subscription<states_msgs::msg::StatesServos>::SharedPtr back_states_sub;

  rclcpp::Subscription<states_msgs::msg::EndState>::SharedPtr end_state_sub;

  rclcpp::Client<board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_pub;
  rclcpp::Client<board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_pub;
  
  rclcpp::Publisher<board_msgs::msg::ServoArray>::SharedPtr front_prop_pub;
  rclcpp::Publisher<board_msgs::msg::ServoArray>::SharedPtr back_prop_pub;
  
  rclcpp::Publisher<board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;

  rclcpp::Publisher<monitor_msgs::msg::DisplayText>::SharedPtr monitor_pub;
};      

} // namespace smov
