#include <configuration/smov_configuration.h>

namespace smov {

front_board_msgs::msg::ServoArray Configuration::front_servo_array;
back_board_msgs::msg::ServoArray Configuration::back_servo_array;

int Configuration::active_board = 1;
int Configuration::rep = 0;
int Configuration::center = 333;
int Configuration::minimum = 83; 
int Configuration::maximum = 520;

// Later on we use std::cin, as it is not problematic to block the loop.
int Configuration::get_char() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           
  newt = oldt; 
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  

  int c = getchar();  // Read character (non-blocking).
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  
  return c;
}

void Configuration::exit_program() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ending program.");
  rclcpp::shutdown();
}

void Configuration::switch_board() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose between the two boards (1: Front board, 2: Back board).");
  std::cin >> Configuration::rep;
  active_board = (int)Configuration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::reset_servos_to(int value, const char* message) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
  for (size_t i = 0; i < SERVO_MAX_SIZE; i++) {
    front_servo_array.servos[i].value = value;
    back_servo_array.servos[i].value = value;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

// Increasing: increase_or_decrease = true.
// Decreasing: increase_or_decrease = false.
void Configuration::increase_or_decrease_by(int value, bool increase_or_decrease, const char* message) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You choose servo number between 1 to 16.");

  std::cin >> Configuration::rep;
  if (increase_or_decrease) {
    front_servo_array.servos[Configuration::rep - 1].value += value;
    back_servo_array.servos[Configuration::rep - 1].value += value;
  } else {
    front_servo_array.servos[Configuration::rep - 1].value -= value;
    back_servo_array.servos[Configuration::rep - 1].value -= value;
  }
}

int Configuration::set_new_value(const char* message) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose a new value : ");

  std::cin >> Configuration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
  return (int)Configuration::rep;
}

} // namespace smov
