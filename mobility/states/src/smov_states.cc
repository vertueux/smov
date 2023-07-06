#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <states/smov_states.h>
#include <states/smov_behaviors.h>

namespace smov {

// Initializing default static values.
RobotStates *RobotStates::instance = nullptr;
FrontServoArray RobotStates::front_prop_servos;
BackServoArray RobotStates::back_prop_servos;
FrontServoArray RobotStates::front_abs_servos;
BackServoArray RobotStates::back_abs_servos;
std::vector<std::vector<long int>> RobotStates::front_servos_data;
std::vector<std::vector<long int>> RobotStates::back_servos_data;
double RobotStates::pulse_for_angle = 0.0;

RobotStates::RobotStates() {}
RobotStates::~RobotStates() {}

RobotStates *RobotStates::Instance() {
  if (!instance) 
    instance = new RobotStates;
  return instance;
}

void RobotStates::on_start() {
  ServoGroupValues val = {0.0, 0.55, 0.85};
  // Default things to test when running the package.
  RobotBehaviors::synch_group_servo_to(val, front_prop_servos, back_prop_servos, BODY_BICEPS_LEGS, 2);
}

void RobotStates::set_up_abs_servos() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_prop_servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.
  }
}

void RobotStates::update_abs_array() {
  RobotStates* node = RobotStates::Instance();

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_abs_array.servos[i] = front_abs_servos[i];
    node->back_abs_array.servos[i] = back_abs_servos[i];
  }
}

void RobotStates::update_prop_array() {
  RobotStates* node = RobotStates::Instance();

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    node->front_prop_array.servos[i] = front_prop_servos[i];
    node->back_prop_array.servos[i] = back_prop_servos[i];
  }
}

} // namespace smov
