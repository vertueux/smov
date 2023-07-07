// Copyright (c) Virtuous. Licensed under the EUPL-1.2 license.
// See LICENSE.md in the project root for license information.

#ifndef SPOT_MICRO_KINEMATICS_H
#define SPOT_MICRO_KINEMATICS_H

#include <kinematics/quadruped_matrix.h>

namespace quadruped {

class QuadrupedKinematics {
 public:
  vector<vector<vector<vector<double>>>> body_kinematics(double omega, double phi, double psi, double xm, double ym, double zm);
  vector<double> leg_kinematics(vector<double> point);
  vector<double> calculate_leg_points(vector<double> angles);

 private:
  double l1 = 50, l2 = 20, l3 = 120, l4 = 155, L = 140, W = 75;
};

}
#endif // SPOT_MICRO_KINEMATICS_H
