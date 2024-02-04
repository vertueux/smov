#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states/robot_manager.h>
#include <states/robot_node_handler.h>

namespace smov {

RobotManager::RobotManager() = default;
RobotManager::~RobotManager() = default;

// Initializing default static values.
RobotManager *RobotManager::instance = nullptr;
RobotManager *RobotManager::Instance() {
  if (!instance)
    instance = new RobotManager;
  return instance;
}

void RobotManager::set_up_servos() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_array.servos[i].servo = static_cast<int16_t>(front_servos_data[i][0] + 1); // Port is at position 0.
    back_prop_array.servos[i].servo = static_cast<int16_t>(back_servos_data[i][0] + 1);  // Servo number = Port + 1.
    front_abs_array.servos[i].servo = static_cast<int16_t>(front_servos_data[i][0] + 1); // Port is at position 0.
    back_abs_array.servos[i].servo = static_cast<int16_t>(back_servos_data[i][0] + 1);   // Servo number = Port + 1.

    front_prop_array.servos[i].value = static_cast<float >(front_servos_data[i][4]); // Port is at position 0.
    back_prop_array.servos[i].value = static_cast<float>(back_servos_data[i][4]);  // Servo number = Port + 1.

    if (RobotNodeHandle::use_single_board) {
      single_back_array.servos[i].servo = static_cast<int16_t>(back_servos_data[i][0] + 1);
      single_back_array.servos[i].value = static_cast<float>(back_servos_data[i][4]);
    }
  }
}
void RobotManager::stop_servos() {
  RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "RobotManager::stop_servos is not implemented");
}
void RobotManager::update_servos_arrays() {
  RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "RobotManager::update_servos_arrays is not implemented");
}

} // namespace smov
