// Copyright (c) Virtuous. Licensed under the GNU AFFERO GENERAL PUBLIC LICENSE.
// See LICENSE in the project root for license information.

#include "kinematics/quadruped_kinematics.h"
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>

template <class T>
std::vector<std::vector<T>> operator+(const std::vector<std::vector<T>> &v1, const std::vector<std::vector<T>> &v2) {
	std::vector<std::vector<T>> ans = v1;
	for (size_t i = 0; i < ans.size(); ++i) {
    for (size_t j = 0; j < ans[i].size(); ++j) {
      ans[i][j] += v2[i][j];
    }
  }
	return ans;
}

template <class T>
std::vector<T> operator+(const std::vector<T> &v1, const std::vector<T> &v2) {
	std::vector<T> ans = v1;
	for (size_t i = 0; i < ans.size(); ++i) {
    ans[i] += v2[i];
  }
	return ans;
}


namespace quadruped {

QuadrupedKinematics::QuadrupedKinematics() {
  l1 = 50;
  l2 = 20; 
  l3 = 120;
  l4 = 155; 
  L = 140; 
  W = 75;
}

std::vector<std::vector<std::vector<std::vector<double>>>> QuadrupedKinematics::body_kinematics(double omega, double phi, double psi, double xm, double ym, double zm) {
  std::vector<std::vector<double>> rx = {{1, 0, 0, 0}, {0, cos(omega), -sin(omega), 0}, {0, sin(omega), cos(omega), 0}, {0, 0, 0, 1}};
  std::vector<std::vector<double>> ry = {{cos(phi), 0, sin(phi), 0}, {0, 1, 0, 0}, {-sin(phi), 0, cos(phi), 0}, {0, 0, 0, 1}};
  std::vector<std::vector<double>> rz = {{cos(psi), -sin(psi), 0, 0}, {sin(psi), cos(psi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  std::vector<std::vector<double>> _rxyz = Matrix::dot_product(ry, rz);
  std::vector<std::vector<double>> rxyz = Matrix::dot_product(rx, _rxyz);
  
  std::vector<std::vector<double>> t{{0, 0, 0, xm}, {0, 0, 0, ym}, {0, 0, 0, zm}, {0, 0, 0, 0}};
  std::vector<std::vector<double>> tm = t + rxyz;

  double s_hp = sin(3.141592653589793 / 2);
  double c_hp = cos(3.141592653589793 / 2);
  double _L = this->L, _W = this->W;

  std::vector<std::vector<double>> _tm1{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  std::vector<std::vector<double>> _tm2{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};
  std::vector<std::vector<double>> _tm3{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  std::vector<std::vector<double>> _tm4{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};

  std::vector<std::vector<double>> new_tm1_rslt = Matrix::dot_product(tm, _tm1);
  std::vector<std::vector<double>> new_tm2_rslt = Matrix::dot_product(tm, _tm2);
  std::vector<std::vector<double>> new_tm3_rslt = Matrix::dot_product(tm, _tm3);
  std::vector<std::vector<double>> new_tm4_rslt = Matrix::dot_product(tm, _tm4);

  std::vector<std::vector<std::vector<std::vector<double>>>> final_result = {{new_tm1_rslt}, {new_tm2_rslt}, {new_tm3_rslt}, {new_tm4_rslt}};
  
  _tm1.clear();
  _tm2.clear();
  _tm3.clear();
  _tm4.clear();

  return final_result;
}

std::vector<double> QuadrupedKinematics::leg_kinematics(std::vector<double> point) {
  double x = point[0], y = point[1], z = point[2];
  double _l1 = l1, _l2 = l2,_l3 = l3,_l4 = l4;
  double f = 0;
  double theta3 = 0;

  if(isnan(sqrt(pow(x, 2) + pow(y, 2) - pow(_l1, 2)))) {
    std::cout << "NAN in the leg kinematics with x: " << x << ", y: " << y << " and l1: " << _l1 << "\n";
    f = _l1;
  } else 
    f = sqrt(pow(x, 2) + pow(y, 2) - pow(l1, 2));

  double g = f - _l2;
  double h = sqrt(pow(g, 2) + pow(z, 2));

  double theta1 =- atan2(y, x) - atan2(f, -_l1);

  double d = ((pow(h, 2) - pow(_l3, 2) - pow(_l4, 2)) / (2 * _l3 * _l4));

  if (isnan(acos(d))) {
    std::cout << "NAN in the leg kinematics with x: " << x << ", y: " << y << " and d: " << d << "\n";
    theta3 = 0;
  } else 
    theta3 = acos(d);

  double theta2 = atan2(z, g) - atan2(_l4 * sin(theta3), _l3 + _l4 * cos(theta3));

  return std::vector<double> {theta1, theta2, theta3};
}

std::vector<double> QuadrupedKinematics::calculate_leg_points(std::vector<double> angles) {
  double _l1 = l1, _l2 = l2, _l3 = l3, _l4 = l4; 
  double theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];
  double theta23 = theta2 + theta3;
  std::vector<double> final_result;

  std::vector<double> t0 = {0, 0, 0, 1};

  for (size_t i = 0; i < t0.size(); i++)
    final_result.push_back(t0.at(i));

  std::vector<double> _t0 = {-_l1 * cos(theta1), _l1 * sin(theta1), 0, 0};
  std::vector<double> t1 = t0 + _t0;
  _t0.clear();

  for (size_t j = 0; j < t1.size(); j++) 
    final_result.push_back(t1.at(j));

  std::vector<double> _t1 = {-_l2 * sin(theta1), -_l2 * cos(theta1), 0, 0};
  std::vector<double> t2 = t1 + _t1;
  _t1.clear();

  for (size_t h = 0; h < t2.size(); h++) 
    final_result.push_back(t2.at(h));
  
  std::vector<double> _t2 = {-_l3 * sin(theta1) * cos(theta2), -_l3 * cos(theta1) * cos(theta2), _l3 * sin(theta2), 0};
  std::vector<double> t3 = t2 + _t2;
  _t2.clear();

  for (size_t k = 0; k < t3.size(); k++) 
    final_result.push_back(t3.at(k));

  std::vector<double> _t3 = {-_l4 * sin(theta1) * cos(theta23), -_l4 * cos(theta1) * cos(theta23), _l4 * sin(theta23), 0};
  std::vector<double> t4 = t3 + _t3;
  _t3.clear();
  
  for (size_t l = 0; l < t4.size(); l++) 
    final_result.push_back(t4.at(l));

  return final_result;
}

} // namespace quadruped

