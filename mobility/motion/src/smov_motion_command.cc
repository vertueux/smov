#include "motion/smov_motion_command.h"

namespace smov {

MotionControl::MotionControl() 
  : Node("motion_control") {
  publisher = this->create_publisher<i2cpwm_board_msgs::msg::ServoArray>("servos_absolute", 1);

  initialize_servos();
}

void MotionControl::initialize_servos() {
  i2cpwm_board_msgs::msg::Servo temp_servo;
  for (int i = 0; i < number_of_servos; i++) {

    temp_servo.servo = i;
    temp_servo.value = 0;
    servo_array.servos.push_back(temp_servo);
  }
}

float smooth_servo_transition(bool up_or_down) {
  // 520 = the maximum.
  if (up_or_down) {
    for (int i = 0; i < 520; i++) {
      return static_cast<float>(i) + 1.0f; 
    }
  } else {
    for (int i = 0; i < 520; i++) {
      return static_cast<float>(i) - 1.0f; 
    }
  }
}

}