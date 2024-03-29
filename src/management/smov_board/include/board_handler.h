//
// Created by ros on 2/3/24.
//

#ifndef BOARD_HANDLER_H_
#define BOARD_HANDLER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "smov_board_msgs/msg/servo_array.hpp"
#include "smov_board_msgs/srv/servos_config.hpp"
#include "smov_board_msgs/srv/drive_mode.hpp"
#include "smov_board_msgs/srv/int_value.hpp"

#include "board_controller.h"

namespace smov {

class BoardHandler {

 public:
  explicit BoardHandler(const std::string &node_name);
  void init(int io_device, int frequency);
  void set_handlers(int board_number);

  void servos_absolute_handler(std::shared_ptr<smov_board_msgs::msg::ServoArray> msg);
  void servos_proportional_handler(std::shared_ptr<smov_board_msgs::msg::ServoArray> msg);
  void servos_drive_handler(std::shared_ptr<geometry_msgs::msg::Twist> msg);
  bool set_pwm_frequency_handler(std::shared_ptr<smov_board_msgs::srv::IntValue::Request> req,
                                 std::shared_ptr<smov_board_msgs::srv::IntValue::Response> res);
  bool config_servos_handler(std::shared_ptr<smov_board_msgs::srv::ServosConfig::Request> req,
                             std::shared_ptr<smov_board_msgs::srv::ServosConfig::Response> res);
  bool config_drive_mode_handler(std::shared_ptr<smov_board_msgs::srv::DriveMode::Request> req,
                                 std::shared_ptr<smov_board_msgs::srv::DriveMode::Response> res);
  bool stop_servos_handler(std::shared_ptr<std_srvs::srv::Empty::Request> req,
                           std::shared_ptr<std_srvs::srv::Empty::Response> res);

  std::shared_ptr<smov::BoardNode> board_node;

 private:

  rclcpp::Service<smov_board_msgs::srv::ServosConfig>::SharedPtr config_srv;
  rclcpp::Subscription<smov_board_msgs::msg::ServoArray>::SharedPtr abs_sub;
  rclcpp::Subscription<smov_board_msgs::msg::ServoArray>::SharedPtr rel_sub;
  rclcpp::Service<smov_board_msgs::srv::IntValue>::SharedPtr freq_srv;
  rclcpp::Service<smov_board_msgs::srv::DriveMode>::SharedPtr mode_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub;

};
}

#endif // BOARD_HANDLER_H_
