#include <manual_walk/robot_manual_walk.h>

namespace smov {

void ManualWalk::init_reader(int echo) {
  fcntl(0, F_SETFL, O_NONBLOCK);
  tcgetattr(0, &old_chars); /* grab old terminal i/o settings */
  new_chars = old_chars; /* make new settings same as old settings */
  new_chars.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_chars.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_chars); /* use these new terminal i/o settings now */
}

void ManualWalk::execute_forward_sequence() {

  front_values.value[RIGHT_BICEPS] =  -0.066f;
  back_values.value[LEFT_BICEPS] =  -0.066f;
  front_values.value[LEFT_BICEPS] = -0.5f;
  back_values.value[RIGHT_BICEPS] = -0.5f;
  front_state_publisher->publish(front_values);
  back_state_publisher->publish(back_values);
  sleep(cooldown);

  front_values.value[LEFT_BICEPS] =  -0.066f;
  back_values.value[RIGHT_BICEPS] =  -0.066f;
  front_values.value[RIGHT_BICEPS] =  -0.5f;
  back_values.value[LEFT_BICEPS] =  -0.5f;
  front_state_publisher->publish(front_values);
  back_state_publisher->publish(back_values);
  sleep(cooldown);

  // Re-executing the same sequence over and over.
  execute_forward_sequence();
}

void ManualWalk::execute_backward_sequence() {
  
}

void ManualWalk::execute_right_sequence() {
  
}

void ManualWalk::execute_left_sequence() {
  
}

void ManualWalk::on_start() {
  init_reader(0);

  // Default value when awakened.
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_values.value[i] = 0.0f;
    front_values.value[i + SERVO_MAX_SIZE / 3] = -0.066f; // -2/3
    front_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;

    back_values.value[i] = 0.0f;
    back_values.value[i + SERVO_MAX_SIZE / 3] = -0.066f;
    back_values.value[i + 2 * SERVO_MAX_SIZE / 3] = 0.12f;
  }
}

void ManualWalk::on_loop() {
  int cmd = getchar();

  switch (cmd) {
    case KEY_UP:
      request_front_walk = true;
      break;
    case KEY_DOWN:
      request_back_walk = true;
      break;
    case KEY_RIGHT:
      request_right_walk = true;
      break;
    case KEY_LEFT:
      request_left_walk = true;
      break;
  }

  if (request_front_walk == true) 
    execute_forward_sequence();
  else if (request_back_walk == true) 
    execute_backward_sequence();
  else if (request_right_walk == true) 
    execute_right_sequence();
  else if (request_left_walk == true)
    execute_left_sequence(); 
}

void ManualWalk::on_quit() {}
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}