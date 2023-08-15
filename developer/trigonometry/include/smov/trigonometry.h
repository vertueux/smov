#pragma once

#include <iostream>
#include <math.h>
#include <numeric>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace smov {

class TrigonometryState {
 public:
  STATE_LIBRARY_CLASS(TrigonometryState)

  TrigonometryState();

  // Phi is the default angle between the body and the leg, it is usually a 90° angle.
  float phi = 90;
};

} // namespace smov
