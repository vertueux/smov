#include <states/smov_behaviors.h>

namespace smov {

void Behaviors::set_servos_to_center(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = States::front_servos_data[i][1]; // Center value is at position 1.
    b_servos[i].value = States::back_servos_data[i][1];
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Look at this front : %ld", States::front_servos_data[i][0]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Look at this back : %ld", States::back_servos_data[i][0]);
  }
}

void Behaviors::set_servos_to_min(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = States::front_servos_data[i][4]; // Minimum value is at position 4.
    b_servos[i].value = States::back_servos_data[i][4];
  }
}

void Behaviors::set_servos_to_max(FrontServoArray f_servos, BackServoArray b_servos) {
  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = States::front_servos_data[i][5]; // Minimum value is at position 5.
    b_servos[i].value = States::back_servos_data[i][5];
  }
}

void Behaviors::set_fs_ang_from_center(int servo, int angle) {
  // All of the values below are default to a standard MG 996R servo and the Dual Board package.
  // We know the base servo angle at it's center (AVCG center angle = 90).
  // For a 100 absolute value pulse, we go forward for 42°.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Default angle for front servo number %d is at %ld°.", servo, States::front_servos_data[servo][6]);

  if (angle > States::front_servos_data[servo][6]) {
    if (States::front_servos_data[servo][4] < States::front_servos_data[servo][0]) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go higher, we need to lower the absolute value.");
      States::front_servos[servo - 1].value = States::front_servos_data[servo][0] - ((angle * 100) / 42);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go higher, we need to higher the absolute value.");
      States::front_servos[servo - 1].value = States::front_servos_data[servo][0] + ((angle * 100) / 42);
    }
  } else {
    if (States::front_servos_data[servo][4] < States::front_servos_data[servo][0]) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go lower, we need to lower the absolute value.");
      States::front_servos[servo - 1].value = States::front_servos_data[servo][0] + ((angle * 100) / 42);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go lower, we need to higher the absolute value.");
      States::front_servos[servo - 1].value = States::front_servos_data[servo][0] - ((angle * 100) / 42);
    }
  }
}

void Behaviors::set_bs_ang_from_center(int servo, int angle) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Default angle for back servo number %d is at %ld° ", servo, States::back_servos_data[servo][6]);

  if (angle > States::back_servos_data[servo][6]) {
    if (States::back_servos_data[servo][4] < States::back_servos_data[servo][0]) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go higher, we need to lower the absolute value.");
      States::back_servos[servo - 1].value = States::back_servos_data[servo][0] - ((angle * 100) / 42);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go higher, we need to higher the absolute value.");
      States::back_servos[servo - 1].value = States::back_servos_data[servo][0] + ((angle * 100) / 42);
    }
  } else {
    if (States::back_servos_data[servo][4] < States::back_servos_data[servo][0]) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go lower, we need to lower the absolute value.");
      States::back_servos[servo - 1].value = States::back_servos_data[servo][0] + ((angle * 100) / 42);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "To go lower, we need to higher the absolute value.");
      States::back_servos[servo - 1].value = States::back_servos_data[servo][0] - ((angle * 100) / 42);
    }
  }
}

}
