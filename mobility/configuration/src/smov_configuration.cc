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
  exit(0);
}

void Configuration::switch_board() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose between the two boards (1: Front board, 2: Back board).");
  std::cin >> Configuration::rep;
  active_board = Configuration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::reset_servos_to_center() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset all servos to center value.");
  for (size_t i = 0; i < front_servo_array.servos.size() + 1; i++) {
    front_servo_array.servos[i].value = Configuration::center;
    back_servo_array.servos[i].value = Configuration::center;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::reset_servos_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset all servos to 0.");
  for (size_t i = 0; i < front_servo_array.servos.size() + 1; i++) {
    front_servo_array.servos[i].value = 0;
    back_servo_array.servos[i].value = 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::increase_servo_by_one() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Manually increasing a servo by 1.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You choose servo number between 1 to 16.");

  std::cin >> Configuration::rep;
  front_servo_array.servos[Configuration::rep - 1].value += 1;
  back_servo_array.servos[Configuration::rep - 1].value += 1;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::decrease_servo_by_one() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Manually decreasing a servo by 1.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You choose servo number between 1 to 16.");

  std::cin >> Configuration::rep;
  front_servo_array.servos[Configuration::rep - 1].value -= 1;
  back_servo_array.servos[Configuration::rep - 1].value -= 1;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::increase_servo_by_ten() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Manually increasing a servo by 10.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You choose servo number between 1 to 16.");

  std::cin >> Configuration::rep;
  front_servo_array.servos[Configuration::rep - 1].value += 10;
  back_servo_array.servos[Configuration::rep - 1].value += 10;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::decrease_servo_by_ten() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Manually decreasing a servo by 10.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You choose servo number between 1 to 16.");

  std::cin >> Configuration::rep;
  front_servo_array.servos[Configuration::rep - 1].value -= 10;
  back_servo_array.servos[Configuration::rep - 1].value -= 10;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::reset_to_maximum_value() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset all servos to maximum value.");
  for (size_t i = 0; i < front_servo_array.servos.size() + 1; i++) {
    front_servo_array.servos[i].value = Configuration::maximum;
    back_servo_array.servos[i].value = Configuration::maximum;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::reset_to_minimum_value() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset all servos to minimum value.");
  for (size_t i = 0; i < front_servo_array.servos.size() + 1; i++) {
    front_servo_array.servos[i].value = Configuration::minimum;
    back_servo_array.servos[i].value = Configuration::minimum;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::set_new_center_value() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting new center value.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose a new value : ");

  std::cin >> Configuration::rep;
  center = (int)Configuration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::set_new_minimum_value() { 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting new minimum value.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose a new value : ");

  std::cin >> Configuration::rep;
  minimum = (int)Configuration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Configuration::set_new_maximum_value() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting new maximum value.");
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose a new value : ");
  
  std::cin >> Configuration::rep;
  maximum = (int)Configuration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

} // namespace smov
