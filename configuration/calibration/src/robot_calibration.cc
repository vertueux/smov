#include <calibration/robot_calibration.h>

namespace smov {

board_msgs::msg::ServoArray Calibration::front_servo_array;
board_msgs::msg::ServoArray Calibration::back_servo_array;

int Calibration::active_board = 1;
int Calibration::rep = 0;
int Calibration::center = 333;
int Calibration::minimum = 83; 
int Calibration::maximum = 520;

// Later on we use std::cin, as it is not problematic to block the loop.
int Calibration::get_char() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           
  newt = oldt; 
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  

  int c = getchar();  // Read character (non-blocking).
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  
  return c;
}

void Calibration::exit_program() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ending program.");
  rclcpp::shutdown();
}

void Calibration::switch_board() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose between the two boards (1: Front board, 2: Back board).");
  std::cin >> Calibration::rep;
  active_board = (int)Calibration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

void Calibration::reset_all_servos_to(int value, const char* msg) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), msg);
  for (size_t i = 0; i < SERVO_MAX_SIZE; i++) {
    front_servo_array.servos[i].value = value;
    back_servo_array.servos[i].value = value;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

// Increasing: increase_or_decrease = true.
// Decreasing: increase_or_decrease = false.
void Calibration::increase_or_decrease_by(int value, bool increase_or_decrease, const char* msg) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), msg); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "You choose servo number between 1 to 16.");

  std::cin >> Calibration::rep;
  if (increase_or_decrease) {
    front_servo_array.servos[Calibration::rep - 1].value += value;
    back_servo_array.servos[Calibration::rep - 1].value += value;
  } else {
    front_servo_array.servos[Calibration::rep - 1].value -= value;
    back_servo_array.servos[Calibration::rep - 1].value -= value;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
}

int Calibration::set_new_value(const char* msg) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), msg);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Choose a new value : ");

  std::cin >> Calibration::rep;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), message);
  return (int)Calibration::rep;
}

} // namespace smov
