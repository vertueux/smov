#include <smov/trigonometry.h>

namespace smov {

TrigonometryState::TrigonometryState() {
  
}

float TrigonometryState::convert_rad_to_deg(float rad) {
  return (rad * (180.0f / M_PI));
}

void TrigonometryState::move_servo_to_ang(MicroController mc, int servo, float angle) {
  float relative_servo = servo;
  if (mc == BACK) relative_servo += SERVO_MAX_SIZE;

  if (angle > data[relative_servo][1] || angle < -data[relative_servo][1]) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle out of range, max angle=%f", data[relative_servo][1]);
    return;
  }
  float factor = 1 / (data[relative_servo][1] - data[relative_servo][0]);

  if (mc == FRONT) {
    front_servos->value[relative_servo] = angle * factor;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Final result=%f", angle * factor);
    (*front_state_publisher)->publish(*front_servos);
  } else {
    back_servos->value[relative_servo] = angle * factor;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Final result=%f", angle * factor);
    (*back_state_publisher)->publish(*back_servos);
  }
}

Vector3 TrigonometryState::set_leg_to(Vector3 xyz) {
  float z_corr = -sqrt(pow(xyz.z, 2.0) + pow(xyz.y, 2.0));
  float c = sqrt(pow(xyz.x, 2.0) + pow(xyz.z, 2.0));
  float d1 = atan2(xyz.x, z_corr);
  float d2 = acos((pow(c, 2.0) + pow(l1, 2.0) - pow(l2, 2.0)) / (2 * c * l1));

  Vector3 result;
  // Getting the angles.
  result.x = -atan2(xyz.y, xyz.z) + M_PI;
  result.y = d1 + d2;
  result.z = acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(c, 2.0)) / (2 * l1 * l2)) - M_PI;

  return result;
}

void TrigonometryState::set_legs_distance_to(float value) {
  // l1: A, l2: B, value: C.
  // We don't actually need angle A, in any case the triangle has to add up to 180°.
  //float a = acos((pow(l2, 2.0) + pow(value, 2.0) - pow(a, 2.0)) / 2 * l2 * value);
  float b = acos((pow(l1, 2.0) + pow(value, 2.0) - pow(l2, 2.0)) / (2 * l1 * value));
  float c = acos((pow(l1, 2.0) + pow(l2, 2.0) - pow(value, 2.0)) / (2 * l1 * l2));
  float theta = M_PI - c;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Beta angle is=%f", b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Theta angle is=%f", theta);

  /*for (int i = 0; i < SERVO_MAX_SIZE / 3; i++) {
    front_servos->value[i] = 0.0f;
    back_servos->value[i] = 0.0f;
  }*/

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
