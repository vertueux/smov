#pragma once

#define SERVO_MAX_SIZE 16

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <unistd.h>
#include <termios.h>
#include <iostream>

#include "board_msgs/msg/servo_array.hpp"
#include "board_msgs/msg/servo.hpp"
namespace smov {

// A completly static class.
class Calibration {
 public:
  static int get_char();
  static void exit_program();
  static void switch_board();
  static void increase_or_decrease_by(int value, bool increase_or_decrease, const char* msg);
  static void reset_all_servos_to(int value, const char* msg);
  static int set_new_value(const char* msg);

  static constexpr const char* message = "\n"
  "\nEnter one of the following options:\n"
  "-----------------------------------\n\n"
  "0: Quit.\n"
  "1: Switch board (1 or 2).\n"
  "2: Reset all servos to center.\n"
  "3: Set all servos to maximum value.\n"
  "4: Set all servos to minimum value.\n"
  "5: Reset all servos to 0.\n"
  "6: Increase a servo by 2.\n"
  "7: Decrease a servo by 2.\n"
  "8: Increase a servo by 10.\n"
  "9: Decrease a servo by 10.\n"
  "A: Set new center value.\n"
  "B: Set new minimum value.\n"
  "C: Set new maximum value.\n";

  // Setting the front board by default.
  static int active_board;

  static board_msgs::msg::ServoArray front_servo_array;
  static board_msgs::msg::ServoArray back_servo_array;

  // Used for answers.
  static int rep;

  static int center;
  static int minimum;
  static int maximum;
};

} // namespace smov
