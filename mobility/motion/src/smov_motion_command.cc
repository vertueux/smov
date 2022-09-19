#include "motion/smov_motion_command.h"

namespace smov {

MotionControl::MotionControl() 
  : Node("motion_control") {
  publisher = this->create_publisher<i2cpwm_board_msgs::msg::ServoArray>("servos_absolute", 1);
}

void MotionControl::initialize_servos() {
  i2cpwm_board_msgs::msg::Servo temp_servo;
  for (int i = 0; i < number_of_servos; i++) {

    temp_servo.servo = i;
    temp_servo.value = 0;
    servo_array.servos.push_back(temp_servo);
  }
}

}