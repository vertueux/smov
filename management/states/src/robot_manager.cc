#include <iostream>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states/robot_manager.h>

namespace smov {

RobotManager::RobotManager() {}
RobotManager::~RobotManager() {}

// Initializing default static values.
RobotManager *RobotManager::instance = nullptr;
RobotManager *RobotManager::Instance() {
  if (!instance) 
    instance = new RobotManager;
  return instance;
}

void RobotManager::init_reader(int echo) {
  fcntl(0, F_SETFL, O_NONBLOCK);
  tcgetattr(0, &old_chars); /* grab old terminal i/o settings */
  new_chars = old_chars; /* make new settings same as old settings */
  new_chars.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_chars.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_chars); /* use these new terminal i/o settings now */
}

void RobotManager::on_start() {
  init_reader(0);
}

/*double biceps_value = 0.0;
double body_value = 0.0;
double leg_value = 1.0;*/
void RobotManager::on_loop() {
  /*int c = getchar();
  switch (c) {
    case KEY_UP:
      if (biceps_value <= 0.98)  
        biceps_value += (1.0/50.0);
      break;
    case KEY_DOWN:
      if (biceps_value >= -0.98)  
        biceps_value -= (1.0/50.0);
      break;
    case KEY_RIGHT:
      if (body_value <= 0.98)  
        body_value += (1.0/50.0);
      break;
    case KEY_LEFT:
      if (body_value >= -0.98)  
        body_value -= (1.0/50.0);
      break;
    case '8':
      if (leg_value <= 0.98)  
        leg_value += (1.0/50.0);
      break;
    case '2':
      if (leg_value >= -0.98)  
        leg_value -= (1.0/50.0);
      break;
  }
  for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_prop_servos[i + (SERVO_MAX_SIZE / 3)].value = biceps_value;
    back_prop_servos[i + (SERVO_MAX_SIZE / 3)].value = biceps_value;
  }

  front_prop_servos[RIGHT_BODY].value = body_value;
  back_prop_servos[RIGHT_BODY].value = body_value;
  front_prop_servos[LEFT_BODY].value = -body_value;
  back_prop_servos[LEFT_BODY].value = -body_value;

  for (int k = 0; k < SERVO_MAX_SIZE / 3; k++) {
    front_prop_servos[k + 2 * (SERVO_MAX_SIZE / 3)].value = leg_value;
    back_prop_servos[k + 2 * (SERVO_MAX_SIZE / 3)].value = leg_value;
  }*/
}

void RobotManager::on_quit() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);
}

void RobotManager::set_up_servos() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_prop_servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.
    front_abs_servos[i].servo = front_servos_data[i][0] + 1; // Port is at position 0.
    back_abs_servos[i].servo = back_servos_data[i][0] + 1;  // Servo number = Port + 1.

    front_prop_servos[i].value = front_servos_data[i][4]; // Port is at position 0.
    back_prop_servos[i].value = back_servos_data[i][4];  // Servo number = Port + 1.
    front_abs_servos[i].value = front_servos_data[i][4]; // Port is at position 0.
    back_abs_servos[i].value = back_servos_data[i][4];  // Servo number = Port + 1.
  }
}

void RobotManager::update_servos_arrays() {
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_prop_array.servos[i] = front_prop_servos[i];
    back_prop_array.servos[i] = back_prop_servos[i];
    front_abs_array.servos[i] = front_abs_servos[i];
    back_abs_array.servos[i] = back_abs_servos[i];
  }
}
} // namespace smov
