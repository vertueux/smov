#include <smov/trigonometry.h>

namespace smov {

float TrigonometryState::convert_rad_to_deg(float rad) {
  return static_cast<float>((rad * (180.0f / M_PI)));
}

void TrigonometryState::move_servo_to_ang(MicroController mc, int servo, float angle) {
  float relative_servo = servo;
  if (mc == BACK) relative_servo += SERVO_MAX_SIZE;

  if (angle > data[relative_servo][1] || angle < -(data[relative_servo][1] - (2 * data[relative_servo][0]))) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle out of range, must be between [%f, %f].",
                -(data[relative_servo][1] - (2 * data[relative_servo][0])), data[relative_servo][1]);
    return;
  }

  float result = (angle - data[relative_servo][0]) / (data[relative_servo][1] - data[relative_servo][0]);

  if (mc == FRONT) {
    front_servos->value[servo] = result;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Final result=%f", result);
    (*front_state_publisher)->publish(*front_servos);
  } else {
    back_servos->value[servo] = result;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Final result=%f", result);
    (*back_state_publisher)->publish(*back_servos);
  }
}

Vector3 TrigonometryState::set_leg_to(Vector3 xyz) {
}

void TrigonometryState::set_legs_distance_to(float value) {
  // l1: A, l2: B, value: C.
  // We don't actually need angle A, in any case the triangle has to add up to 180°.
  //float a = acos((pow(l2, 2.0) + pow(value, 2.0) - pow(a, 2.0)) / 2 * l2 * value);
  float b = acos((pow(l1, 2.0) + pow(value + leg_width, 2.0) - pow(l2, 2.0)) / (2 * (l1) * (value + leg_width)));
  float c = acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(value + leg_width, 2.0)) / (2 * (l1) * (l2)));
  float theta = M_PI - c;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Beta angle is=%f", b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Theta angle is=%f", theta);

  // Moving the front biceps servos.
  move_servo_to_ang(FRONT, 2, convert_rad_to_deg(b));
  move_servo_to_ang(FRONT, 3, convert_rad_to_deg(b));

  // Moving the front legs servos.
  move_servo_to_ang(FRONT, 4, convert_rad_to_deg(theta));
  move_servo_to_ang(FRONT, 5, convert_rad_to_deg(theta));

  // Moving the back biceps servos.
  move_servo_to_ang(BACK, 2, convert_rad_to_deg(b));
  move_servo_to_ang(BACK, 3, convert_rad_to_deg(b));

  // Moving the back legs servos.
  move_servo_to_ang(BACK, 4, convert_rad_to_deg(theta));
  move_servo_to_ang(BACK, 5, convert_rad_to_deg(theta));
}

}
