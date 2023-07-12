#include <chrono>
#include <thread>

#include <basic/robot_basic.h>

namespace smov {

void BasicState::on_start() {
  set_all_servos_to(1.0f);
  set_front_servo_to(LEFT_BICEPS, 0.0f);
  set_back_servo_to(LEFT_BICEPS, 0.0f);
  set_front_servo_to(RIGHT_BICEPS, 0.0f);
  set_back_servo_to(RIGHT_BICEPS, 0.0f);
}

void BasicState::on_loop() {}

void BasicState::on_quit() {}

void BasicState::set_back_servo_to(RobotParts part, float value) {
  if (value > 1.0f || value < -1.0f) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Value selected is not inside I = [-1.0;1.0], [value=%f].", value);
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting back servo number %d to [value=%f].", part, value);

  back_values.value[part] = value; 
}

void BasicState::set_front_servo_to(RobotParts part, float value) {
  if (value > 1.0f || value < -1.0f) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Value selected is not inside I = [-1.0;1.0], [value=%f].", value);
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front servo number %d to [value=%f].", part, value);

  front_values.value[part] = value; 
}

void BasicState::set_all_servos_to(float value) {
  if (value > 1.0f || value < -1.0f) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Value selected is not inside I = [-1.0;1.0], [value=%f].", value);
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to [value=%f].", value);

  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_values.value[i] = value; 
    back_values.value[i] = value;
  }
}

void BasicState::set_all_servos_to_center() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to center [value=0].");

  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_values.value[i] = 0.0f; 
    back_values.value[i] = 0.0f;
  }
}

void BasicState::set_all_servos_to_min() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to minimum value.");

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_values.value[i] = -1.0f; 
    back_values.value[i] = -1.0f;
  }
}

void BasicState::set_all_servos_to_max() {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to maximum value.");

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_values.value[i] = 1.0f;
    back_values.value[i] = 1.0f;
  }
}

}
