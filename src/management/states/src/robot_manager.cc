#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states/robot_manager.h>
#include <states/robot_node_handler.h>

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

void RobotManager::set_up_servos() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_array.servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_prop_array.servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.
    front_abs_array.servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_abs_array.servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.

    front_prop_array.servos[i].value = front_servos_data[i][4]; // Port is at position 0.
    back_prop_array.servos[i].value = back_servos_data[i][4];  // Servo number = Port + 1.

    if (RobotNodeHandle::use_single_board) {
      single_back_array.servos[i].servo = back_servos_data[i][0] + 1;
      single_back_array.servos[i].value = back_servos_data[i][4];
    }
  }
}

} // namespace smov
