// Copyright (c) Virtuous. Licensed under the MIT license.
// See LICENSE.md in the project root for license information.

#include "kinematics/quadruped_kinematics.h"
#include <vector>
#include <math.h>
#include <iomanip>
#include <stdexcept>
#include <iostream>
using namespace std;

template <class T>
vector<vector<T>> operator+(const vector<vector<T>> &v1, const vector<vector<T>> &v2) {
	vector<vector<T>> ans = v1;
	for (size_t i = 0; i < ans.size(); ++i) {
    for (size_t j = 0; j < ans[i].size(); ++j) {
      ans[i][j] += v2[i][j];
    }
  }
	return ans;
}

template <class T>
vector<T> operator+(const vector<T> &v1, const vector<T> &v2) {
	vector<T> ans = v1;
	for (size_t i = 0; i < ans.size(); ++i) {
    ans[i] += v2[i];
  }
	return ans;
}


namespace quadruped {

vector<vector<vector<vector<double>>>> QuadrupedKinematics::body_kinematics(double omega, double phi, double psi, double xm, double ym, double zm) {
  vector<vector<double>> rx = {{1, 0, 0, 0}, {0, cos(omega), -sin(omega), 0}, {0, sin(omega), cos(omega), 0}, {0, 0, 0, 1}};
  vector<vector<double>> ry = {{cos(phi), 0, sin(phi), 0}, {0, 1, 0, 0}, {-sin(phi), 0, cos(phi), 0}, {0, 0, 0, 1}};
  vector<vector<double>> rz = {{cos(psi), -sin(psi), 0, 0}, {sin(psi), cos(psi), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

  vector<vector<double>> _rxyz = Matrix::dot_product(ry, rz);
  vector<vector<double>> rxyz = Matrix::dot_product(rx, _rxyz);
  
  vector<vector<double>> t{{0, 0, 0, xm}, {0, 0, 0, ym}, {0, 0, 0, zm}, {0, 0, 0, 0}};
  vector<vector<double>> tm = t + rxyz;

  double s_hp = sin(3.141592653589793 / 2);
  double c_hp = cos(3.141592653589793 / 2);
  double _L = this->L, _W = this->W;

  vector<vector<double>> _tm1{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm2{{c_hp, 0, s_hp, _L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm3{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, _W / 2}, {0, 0, 0, 1}};
  vector<vector<double>> _tm4{{c_hp, 0, s_hp, -_L / 2}, {0, 1, 0, 0}, {-s_hp, 0, c_hp, -_W / 2}, {0, 0, 0, 1}};

  vector<vector<double>> new_tm1_rslt = Matrix::dot_product(tm, _tm1);
  vector<vector<double>> new_tm2_rslt = Matrix::dot_product(tm, _tm2);
  vector<vector<double>> new_tm3_rslt = Matrix::dot_product(tm, _tm3);
  vector<vector<double>> new_tm4_rslt = Matrix::dot_product(tm, _tm4);

  vector<vector<vector<vector<double>>>> final_result = {{new_tm1_rslt}, {new_tm2_rslt}, {new_tm3_rslt}, {new_tm4_rslt}};
  
  _tm1.clear();
  _tm2.clear();
  _tm3.clear();
  _tm4.clear();

  return final_result;
}

vector<double> QuadrupedKinematics::leg_kinematics(vector<double> point) {
  double x = point[0], y = point[1], z = point[2];
  double _l1 = l1, _l2 = l2,_l3 = l3,_l4 = l4;
  double f = 0;
  double theta3 = 0;

  if(isnan(sqrt(pow(x, 2) + pow(y, 2) - pow(_l1, 2)))) {
    cout << "NAN in the leg kinematics with x: " << x << ", y: " << y << " and l1: " << _l1 << "\n";
    f = _l1;
  } else {
    f = sqrt(pow(x, 2) + pow(y, 2) - pow(l1, 2));
  }

  double g = f - _l2;
  double h = sqrt(pow(g, 2) + pow(z, 2));

  double theta1 =- atan2(y, x) - atan2(f, -_l1);

  double d = ((pow(h, 2) - pow(_l3, 2) - pow(_l4, 2)) / (2 * _l3 * _l4));

  if (isnan(acos(d))) {
    cout << "NAN in the leg kinematics with x: " << x << ", y: " << y << " and d: " << d << "\n";
    theta3 = 0;
  } else {
    theta3 = acos(d);
  }

  double theta2 = atan2(z, g) - atan2(_l4 * sin(theta3), _l3 + _l4 * cos(theta3));

  return vector<double> {theta1, theta2, theta3};
}

vector<double> QuadrupedKinematics::calculate_leg_points(vector<double> angles) {
  double _l1 = l1, _l2 = l2, _l3 = l3, _l4 = l4; 
  double theta1 = angles[0], theta2 = angles[1], theta3 = angles[2];
  double theta23 = theta2 + theta3;
  vector<double> final_result;

  vector<double> t0 = {0, 0, 0, 1};

  for (int i = 0; i < t0.size(); i++)
    final_result.push_back(t0.at(i));

  vector<double> _t0 = {-_l1 * cos(theta1), _l1 * sin(theta1), 0, 0};
  vector<double> t1 = t0 + _t0;
  _t0.clear();

  for (int j = 0; j < t1.size(); j++) 
    final_result.push_back(t1.at(j));

  vector<double> _t1 = {-_l2 * sin(theta1), -_l2 * cos(theta1), 0, 0};
  vector<double> t2 = t1 + _t1;
  _t1.clear();

  for (int h = 0; h < t2.size(); h++) 
    final_result.push_back(t2.at(h));
  
  vector<double> _t2 = {-_l3 * sin(theta1) * cos(theta2), -_l3 * cos(theta1) * cos(theta2), _l3 * sin(theta2), 0};
  vector<double> t3 = t2 + _t2;
  _t2.clear();

  for (int k = 0; k < t3.size(); k++) 
    final_result.push_back(t3.at(k));

  vector<double> _t3 = {-_l4 * sin(theta1) * cos(theta23), -_l4 * cos(theta1) * cos(theta23), _l4 * sin(theta23), 0};
  vector<double> t4 = t3 + _t3;
  _t3.clear();
  
  for (int l = 0; l < t4.size(); l++) 
    final_result.push_back(t4.at(l));

  return final_result;
}

}

