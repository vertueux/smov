#include <chrono>
#include <thread>

#include <smov/sequencer.h>

namespace smov {

void SequencerState::execute_muscles_sequence(RobotMuscles group, const std::vector<float>& values, int cool_down) {
  switch (group) {
    case BODY:
      for (float i : values) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_servos->value[j] = i;
          back_servos->value[j] = i;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending value: %f", i);
        (*front_state_publisher)->publish(*front_servos);
        (*back_state_publisher)->publish(*back_servos);
        delay(cool_down);
      }
      break;
    case BICEPS:
      for (float i : values) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = i;
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = i;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending value: %f", i);
        (*front_state_publisher)->publish(*front_servos);
        (*back_state_publisher)->publish(*back_servos);
        delay(cool_down);
      }
      break;
    case LEGS:
      for (float i : values) {
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++) {
          front_servos->value[j + 2 * (SERVO_MAX_SIZE / 3)] = i;
          back_servos->value[j + 2 * (SERVO_MAX_SIZE / 3)] = i;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending value: %f", i);
        (*front_state_publisher)->publish(*front_servos);
        (*back_state_publisher)->publish(*back_servos);
        delay(cool_down);
      }
      break;
  }
}

void SequencerState::execute_sequence(MicroController mc, int servo, const std::vector<float>& values, int cool_down) {
  if (mc == FRONT) {
    for (float value : values) {
      front_servos->value[servo] = value;
      (*front_state_publisher)->publish(*front_servos);
      delay(cool_down);
    }
  } else {
    for (float value : values) {
      back_servos->value[servo] = value;
      (*back_state_publisher)->publish(*back_servos);
      delay(cool_down);
    }
  }
}

void SequencerState::execute_sequence(MicroController mc,
                                      std::array<int, 2> servos,
                                      const std::vector<float>& values,
                                      int cool_down) {
  if (mc == FRONT) {
    for (float value : values) {
      front_servos->value[servos[0]] = value;
      front_servos->value[servos[1]] = value;
      (*front_state_publisher)->publish(*front_servos);
      delay(cool_down);
    }
  } else {
    for (float value : values) {
      back_servos->value[servos[0]] = value;
      back_servos->value[servos[1]] = value;
      (*back_state_publisher)->publish(*back_servos);
      delay(cool_down);
    }
  }
}

void SequencerState::execute_group_sequence(MicroController mc,
                                            ServoOrder sequence,
                                            std::array<float, 3> values,
                                            int cooldown) {
  for (float i : values) {
    if (i < -1.0 || i > 1.0) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Invalid values that are superior or inferior than I = [-1.0;1.0].");
      return;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Executing sequence with values: [%f, %f, %f]",
              values[0],
              values[1],
              values[2]);

  // The order remains the same. On the group, body servos are on [0;1], 
  // biceps are on [2;3] and legs are on [4;5]. 

  if (mc == FRONT) {
    switch (sequence) {
      case BODY_BICEPS_LEGS:
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          front_servos->value[i] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          front_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);
        break;
      case BODY_LEGS_BICEPS:
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          front_servos->value[i] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          front_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);
        break;
      case BICEPS_LEGS_BODY:
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          front_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          front_servos->value[i] = values.at(2);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);
        break;
      case BICEPS_BODY_LEGS:
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          front_servos->value[i] = values.at(1);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          front_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);
        break;
      case LEGS_BODY_BICEPS:
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          front_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          front_servos->value[i] = values.at(1);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);
        break;
      case LEGS_BICEPS_BODY:
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          front_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          front_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          front_servos->value[i] = values.at(2);
        (*front_state_publisher)->publish(*front_servos);
        delay(cooldown);
        break;
    }
  } else {
    switch (sequence) {
      case BODY_BICEPS_LEGS:
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          back_servos->value[i] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          back_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);
        break;
      case BODY_LEGS_BICEPS:
        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          back_servos->value[i] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          back_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);
        break;
      case BICEPS_LEGS_BODY:
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          back_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          back_servos->value[i] = values.at(2);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);
        break;
      case BICEPS_BODY_LEGS:
        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          back_servos->value[i] = values.at(1);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          back_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);
        break;
      case LEGS_BODY_BICEPS:
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          back_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          back_servos->value[i] = values.at(1);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);
        break;
      case LEGS_BICEPS_BODY:
        for (int h = 0; h < SERVO_MAX_SIZE / 3; h++)
          back_servos->value[h + 2 * (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int j = 0; j < SERVO_MAX_SIZE / 3; j++)
          back_servos->value[j + (SERVO_MAX_SIZE / 3)] = values.at(0);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);

        for (int i = 0; i < SERVO_MAX_SIZE / 3; i++)
          back_servos->value[i] = values.at(2);
        (*back_state_publisher)->publish(*back_servos);
        delay(cooldown);
        break;
    }
  }
}

void SequencerState::execute_global_sequence(MicroController mc, std::vector<float> values, int cool_down) {
  if (mc == FRONT) {
    for (size_t i = 0; i < values.size(); i++) {
      front_servos->value[i] = values[i];
      (*front_state_publisher)->publish(*front_servos);
      delay(cool_down);
    }
  } else {
    for (size_t i = 0; i < values.size(); i++) {
      back_servos->value[i] = values[i];
      (*back_state_publisher)->publish(*back_servos);
      delay(cool_down);
    }
  }
}

}
