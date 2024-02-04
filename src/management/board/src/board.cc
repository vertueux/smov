//
// Created by ros on 2/3/24.
//

#include <cerrno>
#include <unistd.h>
#include <climits>  // for INT_MAX, INT_MIN.
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/empty.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "board_msgs/msg/servo_array.hpp"
#include "board_msgs/srv/servos_config.hpp"
#include "board_msgs/srv/drive_mode.hpp"
#include "board_msgs/srv/int_value.hpp"

#include "board_handler.h"

int main(int argc, char **argv) {

  int io_device = 0;
  rclcpp::init(argc, argv);

  auto board_handler = std::make_shared<smov::BoardHandler>("smov_board");

  rclcpp::Service<board_msgs::srv::ServosConfig>::SharedPtr config_srv;
  rclcpp::Subscription<board_msgs::msg::ServoArray>::SharedPtr abs_sub;
  rclcpp::Subscription<board_msgs::msg::ServoArray>::SharedPtr rel_sub;
  rclcpp::Service<board_msgs::srv::IntValue>::SharedPtr freq_srv;
  rclcpp::Service<board_msgs::srv::DriveMode>::SharedPtr mode_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_sub;

  char *p;
  errno = 0;
  if (argv[1] != nullptr) {
    long conv = strtol(argv[1], &p, 10);
    if (errno != 0 || *p != '\0' || conv > INT_MAX || conv < INT_MIN)
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Expecting an integer to open the i2c-n/ address.");
    else
      io_device = static_cast<int>(conv);
  } else
    io_device = 1;

  auto node = std::make_shared<smov::BoardNode>("smov_board");

  board_handler->set_handlers(io_device);

  board_handler->init(io_device, 50);    // Loads parameters and performs initialization.

  rclcpp::spin(node);

  close(io_device);
  rclcpp::shutdown();
}
