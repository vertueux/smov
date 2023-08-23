#include <iostream>
#include <vector>
#include <iomanip>
#include <stdexcept>

#include <smov/mathematics.h>

#include <kinematics_testing/kinematics_testing.h>

float l1 = 10, l2 = 10;

namespace smov {

float convert_rad_to_deg(float rad) {
  float pi = 3.14159f;
  return(rad * (180.0f / pi));
}

Vector3 leg_kinematics(Vector3 xyz) {
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

void KinematicsTesting::on_start() {
  Vector3 coord = {20, 300, 30};
  Vector3 xyz = leg_kinematics(coord);

  std::cout << convert_rad_to_deg(xyz.x) << std::endl;
  std::cout << convert_rad_to_deg(xyz.y) << std::endl;
  std::cout << convert_rad_to_deg(xyz.z) << std::endl;
}

void KinematicsTesting::on_loop() {
}

void KinematicsTesting::on_quit() {
}

}

DECLARE_STATE_NODE_CLASS("smov_kinematics_testing_state", smov::KinematicsTesting, 10ms)
