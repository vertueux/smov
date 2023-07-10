#include <chrono>
#include <thread>

#include <states/robot_basic_state.h>

namespace smov {

void BasicState::set_servos_to(double value) {
  if (value > 1.0 || value < -1.0) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Value selected is not inside I = [-1.0;1.0], [value=%f].", value);
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to center [value=%f].", value);

  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    (*get_robot())->front_prop_servos[i].value = value; 
    (*get_robot())->back_prop_servos[i].value = value;
  }
}

void BasicState::set_servos_to_center() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to center [value=0].");

  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    (*get_robot())->front_prop_servos[i].value = 0.5; 
    (*get_robot())->back_prop_servos[i].value = 0.5;
  }
}

void BasicState::set_servos_to_min() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to minimum value.");

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    if ((*get_robot())->front_servos_data[i][6] == 1 || (*get_robot())->back_servos_data[i][6] == 1)
      return;

    (*get_robot())->front_prop_servos[i].value = (*get_robot())->front_servos_data[i][6]; 
    (*get_robot())->back_prop_servos[i].value = (*get_robot())->back_servos_data[i][6];
  }
}

void BasicState::set_servos_to_max() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to maximum value.");

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    if ((*get_robot())->front_servos_data[i][7] == 1 || (*get_robot())->back_servos_data[i][7] == 1)
      return;
      
    (*get_robot())->front_prop_servos[i].value = (*get_robot())->front_servos_data[i][7];
    (*get_robot())->back_prop_servos[i].value = (*get_robot())->back_servos_data[i][7];
  }
}

}
