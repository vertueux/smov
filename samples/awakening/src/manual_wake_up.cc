#include <time.h>   
#include <unistd.h>

#include <manual_wake_up/manual_wake_up.h>

namespace smov {

void ManualWakeUpState::sleep_in_milliseconds(int time) {
  ts.tv_sec = time / 1000;
  ts.tv_nsec = (time % 1000) * 1000000;
  nanosleep(&ts, NULL);
}  

void ManualWakeUpState::on_start() {
  // We add a tiny cooldown for safety & to make sure
  // it does the command succesfully.
  sleep_in_milliseconds(cooldown);
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos.value[i] = 0.0f;
    front_servos.value[i + SERVO_MAX_SIZE / 3] = 0.0f;
    front_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 1.0f;
    back_servos.value[i] = 1.0f;
    back_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 1.0f;
    back_servos.value[i + SERVO_MAX_SIZE / 3] = 1.0f;
  }
  
  // We publish the values here as the main node is being locked.
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed first sequence to wake up.");

  // To mark a transition.
  sleep_in_milliseconds(cooldown);

  // Doing the separate waking up phase to the back servos.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
    back_servos.value[i] = 0.0f;
  back_state_publisher->publish(back_servos);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed second sequence to wake up.");

  // One more transition.
  sleep_in_milliseconds(cooldown);

  // Executing the last sequence.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f; // -2/3
    front_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    back_servos.value[i + SERVO_MAX_SIZE / 3] = -0.066f;
    back_servos.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot may have woken up!");
}

void ManualWakeUpState::on_loop() {}

void ManualWakeUpState::on_quit() {}

}

DECLARE_STATE_NODE_CLASS("smov_legacy_awakening_state", smov::ManualWakeUpState, 500ms)
