#ifndef ROBOT_NODE_HANDLER_H_
#define ROBOT_NODE_HANDLER_H_

#include <states/robot_manager.h>

#include <std_srvs/srv/empty.hpp>

#include "smov_board_msgs/srv/servos_config.hpp"
#include "smov_board_msgs/msg/servo_config.hpp"
#include "smov_states_msgs/msg/states_servos.hpp"
#include "smov_states_msgs/msg/end_state.hpp"
#include "smov_monitor_msgs/msg/display_text.hpp"

namespace smov {

class RobotNodeHandle : public rclcpp::Node {
 public:
  RobotNodeHandle();

  static bool use_single_board;

  void declare_parameters();
  void set_up_topics();
  void config_servos();
  void late_callback();
  void front_topic_callback(smov_states_msgs::msg::StatesServos::SharedPtr msg);
  void back_topic_callback(smov_states_msgs::msg::StatesServos::SharedPtr msg);
  void end_state_callback(smov_states_msgs::msg::EndState::SharedPtr msg);
  void stop_servos();

  // Used for fast operations
  rclcpp::TimerBase::SharedPtr timer;

  // Used for non-necessary fast operations
  rclcpp::TimerBase::SharedPtr late_timer;

  // Used to publish on the LCD panel.
  smov_monitor_msgs::msg::DisplayText up_display;

  // Used to publish on the LCD panel.
  std::string up_display_str;

  // Creating the base robot with all the necessary data & publishers.
  RobotManager *robot = RobotManager::Instance();

  rclcpp::Subscription<smov_states_msgs::msg::StatesServos>::SharedPtr front_states_sub;
  rclcpp::Subscription<smov_states_msgs::msg::StatesServos>::SharedPtr back_states_sub;

  rclcpp::Subscription<smov_states_msgs::msg::EndState>::SharedPtr end_state_sub;

  rclcpp::Client<smov_board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_client;
  rclcpp::Client<smov_board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_client;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr front_stop_servos_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr back_stop_servos_client;

  rclcpp::Publisher<smov_board_msgs::msg::ServoArray>::SharedPtr front_prop_pub;
  rclcpp::Publisher<smov_board_msgs::msg::ServoArray>::SharedPtr back_prop_pub;

  rclcpp::Publisher<smov_board_msgs::msg::ServoArray>::SharedPtr front_abs_pub;
  rclcpp::Publisher<smov_board_msgs::msg::ServoArray>::SharedPtr back_abs_pub;

  rclcpp::Publisher<smov_monitor_msgs::msg::DisplayText>::SharedPtr monitor_pub;
};

} // namespace smov

#endif // ROBOT_NODE_HANDLER_H_
