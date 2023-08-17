#include <smov/trigonometry.h>

namespace smov {

TrigonometryState::TrigonometryState() {
  
}

Vector3 TrigonometryState::leg_kinematics(Vector3 xyz) {
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

}
