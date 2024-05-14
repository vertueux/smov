#ifndef ROBOT_H_
#define ROBOT_H_

#define SERVO_MAX_SIZE 6

#include <chrono>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>

#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/msg/servo.hpp"
#include "i2c_pwm_board_msgs/srv/servos_config.hpp"
#include "i2c_pwm_board_msgs/msg/servo_config.hpp"

#include "smov_states_msgs/msg/states_servos.hpp"
#include "smov_states_msgs/msg/end_state.hpp"
#include "smov_monitor_msgs/msg/display_text.hpp"

namespace smov {

struct RobotData {
  static RobotData *Instance();
  RobotData(const RobotData &) = delete;
  RobotData &operator=(const RobotData &) = delete;

  static bool use_single_board;
  static double upper_leg_length;
  static double lower_leg_length;
  static double hip_body_distance;

  // Arrays to publish in the proportional publisher.
  i2c_pwm_board_msgs::msg::ServoArray front_prop_array;
  i2c_pwm_board_msgs::msg::ServoArray back_prop_array;

  // When using a single board.
  i2c_pwm_board_msgs::msg::ServoArray single_back_array;

  std::vector<std::vector<long int>> front_servos_data;
  std::vector<std::vector<long int>> back_servos_data;

  std::string state = "None";

  // Used to publish on the LCD panel.
  smov_monitor_msgs::msg::DisplayText up_display;

  std::array<std::string, 12> servo_name = {
    "FRONT_BODY_LEFT",       "FRONT_BODY_RIGHT",     "FRONT_UPPER_LEG_LEFT",
    "FRONT_UPPER_LEG_RIGHT", "FRONT_LOWER_LEG_LEFT", "FRONT_LOWER_LEG_RIGHT",
    "BACK_BODY_LEFT",        "BACK_BODY_RIGHT",      "BACK_UPPER_LEG_LEFT",
    "BACK_UPPER_LEG_RIGHT",  "BACK_LOWER_LEG_LEFT",  "BACK_LOWER_LEG_RIGHT"
  };

 private:
  static RobotData *instance;
  RobotData();
  ~RobotData();
};

class RobotNodeHandle : public rclcpp::Node {
 public:
  RobotNodeHandle();

  // Creating the base robot with all the necessary data & publishers.
  RobotData *robot = RobotData::Instance();

  void declare_parameters();
  void set_up_servos();
  void set_up_topics();
  void config_servos();
  void output_values();
  void front_topic_callback(smov_states_msgs::msg::StatesServos::SharedPtr msg);
  void back_topic_callback(smov_states_msgs::msg::StatesServos::SharedPtr msg);
  void end_state_callback(smov_states_msgs::msg::EndState::SharedPtr msg);
  void stop_servos();

  // Used for non-necessary fast operations
  rclcpp::TimerBase::SharedPtr late_timer;

  // Used to publish on the LCD panel.
  std::string up_display_str;

  rclcpp::Subscription<smov_states_msgs::msg::StatesServos>::SharedPtr front_states_sub;
  rclcpp::Subscription<smov_states_msgs::msg::StatesServos>::SharedPtr back_states_sub;

  rclcpp::Subscription<smov_states_msgs::msg::EndState>::SharedPtr end_state_sub;

  rclcpp::Client<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr front_servo_config_client;
  rclcpp::Client<i2c_pwm_board_msgs::srv::ServosConfig>::SharedPtr back_servo_config_client;

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr front_stop_servos_client;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr back_stop_servos_client;

  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr front_prop_pub;
  rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr back_prop_pub;

  rclcpp::Publisher<smov_monitor_msgs::msg::DisplayText>::SharedPtr monitor_pub;
};


} // namespace smov

#endif // ROBOT_H_
