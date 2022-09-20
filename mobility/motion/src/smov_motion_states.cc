#include <chrono>
#include <thread>

#include "motion/smov_motion_states.h"
#include "motion/smov_motion_command.h"


// From this resonance, we assume that the servo motors 
// of the robot start from right to left, top to bottom, 
// which allows us to move them independently.

namespace smov {

void StateAlgorithms::walk(i2cpwm_board_msgs::msg::ServoArray servo_array) {
  state.walking = true;
  
  // We can directly put number of servos / 4 as the quadruped robot contains 
  // 4 legs with 3 servos each one.
  for (int i = 0; i < (MotionControl::number_of_servos / 3); i++) {
    for (int j = 0; j < (MotionControl::number_of_servos / 4); j++) {
      if (j == 0|| j == 3 || j == 6 || j == 9) 
        j = 0;

      servo_array.servos[j * i].value = MotionControl::smooth_servo_transition(true) / 2.0f;
      servo_array.servos[j * i].value = MotionControl::smooth_servo_transition(true);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    servo_array.servos[i * i + 1].value = MotionControl::smooth_servo_transition(false) / 2.0f;
    servo_array.servos[i * i + 1].value = MotionControl::smooth_servo_transition(false);
  }
}

void StateAlgorithms::idle(i2cpwm_board_msgs::msg::ServoArray servo_array) {
  for (int i = 0; i < MotionControl::number_of_servos; i++) {
    servo_array.servos[i].value = 0.0f;
  }
}

}