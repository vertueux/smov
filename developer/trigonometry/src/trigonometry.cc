#include <chrono>
#include <thread>
#include <math.h>
#include <Eigen/Dense>

#include <smov/trigonometry.h>

namespace smov {

TrigonometryState::TrigonometryState(float _l1, float _l2, float _l3) {
  l1 = _l1;
  l2 = _l2;
  l3 = _l3;
}

void TrigonometryState::solve_leg_kinematics(float x, float y, float z, bool is_right) {
  float a = sqrt(pow(z, 2.0) + pow(z, 2.0));
  float a_ang_1 = sin(90) / a;
  float a_ang_2 = asin((l1/a) * sin(90));
  float a_ang_3 = 90 - a_ang_2;
  float theta_1 = 0;

  if (is_right)
    theta_1 = a_ang_1 - a_ang_3;
  else 
    theta_1 = a_ang_1 + a_ang_3;
}

}
