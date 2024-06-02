#include "points_and_functions.h"

void PointsAndFunctionsState::init_reader(int echo) {
  fcntl(0, F_SETFL, O_NONBLOCK);
  tcgetattr(0, &old_chars);                 // Grab old terminal i/o settings. 
  new_chars = old_chars;                    // Make new settings same as old settings. 
  new_chars.c_lflag &= ~ICANON;             // Disable buffered i/o. 
  new_chars.c_lflag &= echo ? ECHO : ~ECHO; // Set echo mode. 
  tcsetattr(0, TCSANOW, &new_chars);        // Use these new terminal i/o settings now.
}
  
void PointsAndFunctionsState::on_start() {
  init_reader(0);

  for (int i = 0; i < 6; i++) {
    front_servos.value[i] = 0.0f;
  }

  front_state_publisher->publish(front_servos);

  front_servos.value[1] = 90.0f;
  front_servos.value[3] = 55.0f;
  front_servos.value[5] = 30.0f; // (val - base) / (max - base) = -1

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting default position to (6, 12, 12)");
  coord.x = 6;
  coord.y = 12;
  coord.z = 12;
  trig.set_leg_to(1, coord);
}

void PointsAndFunctionsState::on_loop() {
  // This will called every 10ms.
  float sensibility = 10.0f;
  int c = getchar();
  switch (c) {
    case KEY_UP:
      coord.x += (1.0 / sensibility);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
      trig.set_leg_to(1, coord);
      break;
    case KEY_DOWN:
      coord.x -= (1.0 / sensibility);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
      trig.set_leg_to(1, coord);
      break;
    case KEY_RIGHT:
      coord.z += (1.0 / sensibility); 
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
      trig.set_leg_to(1, coord);
      break;
    case KEY_LEFT:
      coord.z -= (1.0 / sensibility);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
      trig.set_leg_to(1, coord);
      break;
    case 'e':
      coord.y += (1.0 / sensibility);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
      trig.set_leg_to(1, coord);
      break;
    case 'c':
      coord.y -= (1.0 / sensibility);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[2J\033[;H");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Coordinates: (%f, %f, %f)", coord.x, coord.y, coord.z);
      trig.set_leg_to(1, coord);
      break;
  }
  
  front_state_publisher->publish(front_servos);
}

void PointsAndFunctionsState::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("points_and_functions", PointsAndFunctionsState, 10ms)
