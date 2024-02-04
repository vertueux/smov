//
// Created by ros on 2/3/24.
//

#ifndef BOARD_HANDLER_H_
#define BOARD_HANDLER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "board_msgs/msg/servo_array.hpp"
#include "board_msgs/srv/servos_config.hpp"
#include "board_msgs/srv/drive_mode.hpp"
#include "board_msgs/srv/int_value.hpp"

#include "board_controller.h"

namespace smov {

class BoardHandler {

 public:
  explicit BoardHandler(const std::string &node_name);
  void init(int io_device, int frequency);
  void set_handlers(int board_number);

  void servos_absolute_handler(std::shared_ptr<board_msgs::msg::ServoArray> msg);
  void servos_proportional_handler(std::shared_ptr<board_msgs::msg::ServoArray> msg);
  void servos_drive_handler(std::shared_ptr<geometry_msgs::msg::Twist> msg);
  bool set_pwm_frequency_handler(std::shared_ptr<board_msgs::srv::IntValue::Request> req,
                                 std::shared_ptr<board_msgs::srv::IntValue::Response> res);
  bool config_servos_handler(std::shared_ptr<board_msgs::srv::ServosConfig::Request> req,
                             std::shared_ptr<board_msgs::srv::ServosConfig::Response> res);
  bool config_drive_mode_handler(std::shared_ptr<board_msgs::srv::DriveMode::Request> req,
                                 std::shared_ptr<board_msgs::srv::DriveMode::Response> res);
  bool stop_servos_handler(std::shared_ptr<std_srvs::srv::Empty::Request> req,
                           std::shared_ptr<std_srvs::srv::Empty::Response> res);

 private:
  std::shared_ptr<smov::BoardNode> board_node;
};
}

#endif // BOARD_HANDLER_H_
