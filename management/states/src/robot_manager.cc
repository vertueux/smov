#include <iostream>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states/robot_manager.h>

namespace smov {

RobotManager::RobotManager() {}
RobotManager::~RobotManager() {}

// Initializing default static values.
RobotManager *RobotManager::instance = nullptr;
RobotManager *RobotManager::Instance() {
  if (!instance) 
    instance = new RobotManager;
  return instance;
}

void RobotManager::init_reader(int echo) {
  fcntl(0, F_SETFL, O_NONBLOCK);
  tcgetattr(0, &old_chars); /* grab old terminal i/o settings */
  new_chars = old_chars; /* make new settings same as old settings */
  new_chars.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_chars.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_chars); /* use these new terminal i/o settings now */
}

void RobotManager::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

void RobotManager::set_up_servos() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_array.servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_prop_array.servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.
    front_abs_array.servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_abs_array.servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.

    front_prop_array.servos[i].value = front_servos_data[i][4]; // Port is at position 0.
    back_prop_array.servos[i].value = back_servos_data[i][4];  // Servo number = Port + 1.
  }
}

} // namespace smov
