#include <states/smov_behaviors.h>
namespace smov {

time_t RobotBehaviors::counter = time(0);

void RobotBehaviors::synch_front_group_servo_to(ServoGroupValues values, FrontServoArray group, ServoOrder sequence, time_t timeout) {
  for (size_t i = 0; i < values.size();i++) {
    if (values[i] < -1.0 || values[i] > 1.0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
        "Invalid ServoGroupValues value that is superior or inferior than I = [-1.0;1.0].");
      return;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing front sequence with values: [%f, %f, %f]", values[0], values[1], values[2]);

  // The order remains the same. On the group, body servos are on [0;1], 
  // biceps are on [2;3] and legs are on [4;5]. 
  switch (sequence) {
    case BODY_BICEPS_LEGS: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case BODY_LEGS_BICEPS:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case BICEPS_LEGS_BODY:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j  + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h].value = values.at(2);
      }
      break;
    case BICEPS_BODY_LEGS: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case LEGS_BODY_BICEPS:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case LEGS_BICEPS_BODY: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h].value = values.at(2);
      }
      break;
  }
}

void RobotBehaviors::synch_back_group_servo_to(ServoGroupValues values, BackServoArray group, ServoOrder sequence, time_t timeout) {
  for (size_t i = 0; i < values.size();i++) {
    if (values[i] < -1.0 || values[i] > 1.0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
        "Invalid ServoGroupValues value that is superior or inferior than I = [-1.0;1.0].");
      return;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing back sequence with values: [%f, %f, %f]", values[0], values[1], values[2]);

  switch (sequence) {
    case BODY_BICEPS_LEGS: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case BODY_LEGS_BICEPS:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case BICEPS_LEGS_BODY:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j  + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h].value = values.at(2);
      }
      break;
    case BICEPS_BODY_LEGS: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case LEGS_BODY_BICEPS:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h + (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      break;
    case LEGS_BICEPS_BODY: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) 
        group[i + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(0);
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) 
          group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) 
          group[h].value = values.at(2);
      }
      break;
  }
}

void RobotBehaviors::synch_group_servo_to(ServoGroupValues values, FrontServoArray front_group, BackServoArray back_group, ServoOrder sequence, time_t timeout) {
  for (size_t i = 0; i < values.size();i++) {
    if (values[i] < -1.0 || values[i] > 1.0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
        "Invalid ServoGroupValues value that is superior or inferior than I = [-1.0;1.0].");
      return;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing dual sequence with values: [%f, %f, %f]", values[0], values[1], values[2]);

  switch (sequence) {
    case BODY_BICEPS_LEGS: 
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
        front_group[i].value = values.at(0);
        back_group[i].value = values.at(0);
      }
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
          back_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
        }
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) {
          front_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
          back_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
        }
      }
      break;
    case BODY_LEGS_BICEPS:
      for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
        front_group[i].value = values.at(0);
        back_group[i].value = values.at(0);
      }
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) {
          front_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
          back_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
        }
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
          back_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
        }
      }
      break;
    case BICEPS_LEGS_BODY:
      for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
        front_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
        back_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) {
          front_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
          back_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
        }
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
          front_group[i].value = values.at(0);
          back_group[i].value = values.at(0);
        }
      }
      break;
    case BICEPS_BODY_LEGS: 
      for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
        front_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
        back_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
      }
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
          front_group[i].value = values.at(0);
          back_group[i].value = values.at(0);
        }
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) {
          front_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
          back_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
        }
      }
      break;
    case LEGS_BODY_BICEPS:
      for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) {
          front_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
          back_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
          front_group[i].value = values.at(0);
          back_group[i].value = values.at(0);
        }
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
          back_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
        }
      }
      break;
    case LEGS_BICEPS_BODY: 
      for (int h = 0; h < SERVO_MAX_SIZE / 3; h++) {
        front_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
        back_group[h + 2 * (SERVO_MAX_SIZE / 3)].value = values.at(2);
      }
      if (time(0) - RobotBehaviors::counter > timeout) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
          back_group[j + (SERVO_MAX_SIZE / 3)].value = values.at(1);
        }
      }
      if (time(0) - RobotBehaviors::counter > timeout * 2) {
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
          front_group[i].value = values.at(0);
          back_group[i].value = values.at(0);
        }
      }
      break;
  }
}

void RobotBehaviors::set_servos_to_center(FrontServoArray f_servos, BackServoArray b_servos) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to center [value=0].");

  // Taking the values from the YAML file.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    f_servos[i].value = 0; 
    b_servos[i].value = 0;
  }
}

void RobotBehaviors::set_servos_to_min(FrontServoArray f_servos, BackServoArray b_servos) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to minimum value.");

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    if (RobotStates::front_servos_data[i][5] == 1 || RobotStates::back_servos_data[i][5] == 1)
      return;

    f_servos[i].value = RobotStates::front_servos_data[i][5]; 
    b_servos[i].value = RobotStates::back_servos_data[i][5];
  }
}

void RobotBehaviors::set_servos_to_max(FrontServoArray f_servos, BackServoArray b_servos) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting front & back servos to maximum value.");

  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    if (RobotStates::front_servos_data[i][6] == 1 || RobotStates::back_servos_data[i][6] == 1)
      return;
      
    f_servos[i].value = RobotStates::front_servos_data[i][6];
    b_servos[i].value = RobotStates::back_servos_data[i][6];
  }
}

}
