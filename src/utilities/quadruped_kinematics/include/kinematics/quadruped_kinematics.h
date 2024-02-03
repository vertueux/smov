// Copyright (c) Virtuous. Licensed under the GNU AFFERO GENERAL PUBLIC LICENSE.
// See LICENSE in the project root for license information.

#ifndef QUADRUPED_KINEMATICS_H
#define QUADRUPED_KINEMATICS_H

#include <kinematics/quadruped_matrix.h>

namespace quadruped {

class QuadrupedKinematics {
 public:
  QuadrupedKinematics();

  std::vector<std::vector<std::vector<std::vector<double>>>> body_kinematics(double omega, double phi, double psi, double xm, double ym, double zm);
  std::vector<double> leg_kinematics(std::vector<double> point);
  std::vector<double> calculate_leg_points(std::vector<double> angles);

 private:
  double l1 = 50, l2 = 20, l3 = 120, l4 = 155, L = 140, W = 75;
};

} // namespace quadruped

#endif // QUADRUPED_KINEMATICS_H
