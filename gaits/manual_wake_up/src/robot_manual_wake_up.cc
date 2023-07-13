#include <unistd.h>

#include <manual_wake_up/robot_manual_wake_up.h>

namespace smov {

void ManualWakeUpState::on_start() {
  // We add a tiny cooldown for safety & to make sure
  // it does the command succesfully.
  sleep(cooldown);
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_values.value[i] = 0.0f;
    front_values.value[i + SERVO_MAX_SIZE / 3] = 0.0f;
    front_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 1.0f;
    back_values.value[i] = 1.0f;
    back_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 1.0f;
    back_values.value[i + SERVO_MAX_SIZE / 3] = 1.0f;
  }
  
  // We publish the values here as the main node is being locked.
  front_state_publisher->publish(front_values);
  back_state_publisher->publish(back_values);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed first sequence to wake up.");

  // To mark a transition.
  sleep(cooldown);

  // Doing the separate waking up phase to the back servos.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
    back_values.value[i] = 0.0f;
  back_state_publisher->publish(back_values);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executed second sequence to wake up.");

  // One more transition.
  sleep(cooldown);

  // Executing the last sequence.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_values.value[i + SERVO_MAX_SIZE / 3] = -0.066f; // -2/3
    front_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    back_values.value[i + SERVO_MAX_SIZE / 3] = -0.066f;
    back_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot may have woken up!");
}

void ManualWakeUpState::on_loop() {}

void ManualWakeUpState::on_quit() {}

}