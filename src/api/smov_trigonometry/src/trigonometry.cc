#include <smov/trigonometry.h>

namespace smov {

float TrigonometryState::convert_rad_to_deg(float rad) {
  return static_cast<float>((rad * (180.0f / M_PI)));
}

void TrigonometryState::set_leg_to(int leg_number, Vector3 xyz) {
  // float c = sqrt(pow(xyz.z, 2.0) + pow(xyz.y, 2.0));
  // float d = sqrt(pow(c, 2.0) - pow(shoulder_width, 2.0))
  // Simplified:
  float d = sqrt(pow(xyz.z, 2.0) + pow(xyz.y, 2.0) - pow(shoulder_width, 2.0));
  
  // Calculating the angle.
  // float a = atan(xyz.z / xyz.y);
  // float b = atan(d / shoulder_width);
  // Simplified: 
  float omega = atan(xyz.z / xyz.y) + atan(d / shoulder_width);

  float g = sqrt(pow(d, 2.0) + pow(xyz.x, 2.0));

  // Al-Kashi's theorem.
  float phi = acos((pow(g, 2.0) - pow(l1, 2.0) - pow(l2, 2.0)) / (-2 * l1 * l2));

  float theta = atan(xyz.x / d) + asin((l2 * sin(phi))/ g);

  if (leg_number == 1) {
    front_servos->value[0] = omega;
    front_servos->value[1] = theta;
    front_servos->value[2] = phi;
  } else if (leg_number == 2) {
    front_servos->value[3] = omega;
    front_servos->value[4] = theta;
    front_servos->value[5] = phi;
  } else if (leg_number == 2) {
    back_servos->value[0] = omega;
    back_servos->value[1] = theta;
    back_servos->value[2] = phi;
  } else if (leg_number == 2) {
    back_servos->value[3] = omega;
    back_servos->value[4] = theta;
    back_servos->value[5] = phi;
  }

  // Publishing the values.
  (*front_state_publisher)->publish(*front_servos);
  (*back_state_publisher)->publish(*back_servos);
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

  front_servos->value[2] = convert_rad_to_deg(b);
  front_servos->value[3] = convert_rad_to_deg(b);

  front_servos->value[4] = convert_rad_to_deg(theta);
  front_servos->value[5] = convert_rad_to_deg(theta);

  back_servos->value[2] = convert_rad_to_deg(b);
  back_servos->value[3] = convert_rad_to_deg(b);

  back_servos->value[4] = convert_rad_to_deg(theta);
  back_servos->value[5] = convert_rad_to_deg(theta);

  // Publishing the values.
  (*front_state_publisher)->publish(*front_servos);
  (*back_state_publisher)->publish(*back_servos);
}

}
