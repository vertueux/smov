#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states/robot_manager.h>
#include <states/robot_basic_state.h>

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

void RobotManager::on_start() {
  BasicState basic_state;

  basic_state.set_servos_to(0.2);
}

void RobotManager::on_loop() {}

void RobotManager::set_up_servos() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_prop_servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.
    front_abs_servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_abs_servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.

    front_prop_servos[i].value = front_servos_data[i][5]; // Port is at position 0.
    back_prop_servos[i].value = back_servos_data[i][5];  // Servo number = Port + 1.
    front_abs_servos[i].value = front_servos_data[i][5]; // Port is at position 0.
    back_abs_servos[i].value = back_servos_data[i][5];  // Servo number = Port + 1.
  }
}

void RobotManager::update_servos_arrays() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_array.servos[i] = front_prop_servos[i];
    back_prop_array.servos[i] = back_prop_servos[i];
    front_abs_array.servos[i] = front_abs_servos[i];
    back_abs_array.servos[i] = back_abs_servos[i];
  }
}
} // namespace smov
