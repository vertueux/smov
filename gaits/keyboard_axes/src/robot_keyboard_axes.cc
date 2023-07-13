#include <keyboard_axes/robot_keyboard_axes.h>

namespace smov {

void KeyboardAxesState::init_reader(int echo) {
  fcntl(0, F_SETFL, O_NONBLOCK);
  tcgetattr(0, &old_chars); /* grab old terminal i/o settings */
  new_chars = old_chars; /* make new settings same as old settings */
  new_chars.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_chars.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_chars); /* use these new terminal i/o settings now */
}

void KeyboardAxesState::on_start() {
  // This will be called when the node starts running.
  init_reader(0);
}

void KeyboardAxesState::on_loop() {
  // This will called every 900ms (But you 
  // can change the timeout in ./robot_main.cc).
  int c = getchar();
  switch (c) {
    case KEY_UP:
      if (biceps_value <= 0.98) {
        biceps_value += (1.0/90.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going up with value: %f", biceps_value);
      }
      break;
    case KEY_DOWN:
      if (biceps_value >= -0.98) {
        biceps_value -= (1.0/90.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going down with value: %f", biceps_value);
      }
      break;
    case KEY_RIGHT:
      if (body_value <= 0.98) {
        body_value += (1.0/90.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going right with value: %f", body_value);
      }
      break;
    case KEY_LEFT:
      if (body_value >= -0.98) {
        body_value -= (1.0/90.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Going left with value: %f", body_value);
      }
      break;
    case '8':
      if (leg_value <= 0.98) {
        leg_value += (1.0/90.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Putting the legs up with value: %f", leg_value);
      }
      break;
    case '2':
      if (leg_value >= -0.98) {
        leg_value -= (1.0/90.0);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Putting the legs down with value: %f", leg_value);
      }
      break;
  }
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_values.value[i + (SERVO_MAX_SIZE / 3)] = -biceps_value;
    back_values.value[i + (SERVO_MAX_SIZE / 3)] = -biceps_value;
  }

  front_values.value[RIGHT_BODY] = body_value;
  back_values.value[RIGHT_BODY] = body_value;
  front_values.value[LEFT_BODY] = -body_value;
  back_values.value[LEFT_BODY] = -body_value;

  for (int k = 0; k < SERVO_MAX_SIZE / 3; k++) {
    front_values.value[k + 2 * (SERVO_MAX_SIZE / 3)] = leg_value;
    back_values.value[k + 2 * (SERVO_MAX_SIZE / 3)] = leg_value;
  }
}

void KeyboardAxesState::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

}