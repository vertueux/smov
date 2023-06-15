#pragma once

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#include "front_board_msgs/msg/servo_array.hpp"
#include "front_board_msgs/msg/servo.hpp"
#include "back_board_msgs/msg/servo_array.hpp"
#include "back_board_msgs/msg/servo.hpp"

namespace smov {

// A completly static class.
class Configuration {
 public:
  static int get_char();
  static void exit_program();
  static void switch_board();
  static void reset_servos_to_center();
  static void reset_servos_to_zero();
  static void increase_servo_by_one();
  static void decrease_servo_by_one();
  static void increase_servo_by_ten();
  static void decrease_servo_by_ten();
  static void reset_to_maximum_value();
  static void reset_to_minimum_value();
  static void set_new_center_value();
  static void set_new_minimum_value();
  static void set_new_maximum_value();

  static constexpr const char* message = "\n"
  "\nEnter one of the following options:\n"
  "-----------------------------------\n\n"
  "0: Quit.\n"
  "1: Switch board (1 or 2).\n"
  "2: Reset all servos to center.\n"
  "3: Reset all servos to 0.\n"
  "4: Increase a servo by 1.\n"
  "5: Decrease a servo by 1.\n"
  "6: Increase a servo by 10.\n"
  "7: Decrease a servo by 10.\n"
  "8: Set all servos to maximum value.\n"
  "9: Set all servos to minimum value.\n"
  "A: Set new center value.\n"
  "B: Set new minimum value.\n"
  "C: Set new maximum value.\n";

  // Setting the front board by default.
  static int active_board;

  static front_board_msgs::msg::ServoArray front_servo_array;
  static back_board_msgs::msg::ServoArray back_servo_array;

  // Used for answers.
  static int rep;

  static int center;
  static int minimum;
  static int maximum;
};

} // namespace smov
