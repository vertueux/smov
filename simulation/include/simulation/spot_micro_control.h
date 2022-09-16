#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <kinematics/quadruped_kinematics.h>

using namespace std;

namespace smov {

  constexpr unsigned int str2int(const char* str, int h = 0) {
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
  }

  struct PredefinedPositions {
    vector<double> crouched = {-55, -100, 20};
    vector<double> up = {-55, -190, 20};
    vector<double> sit_front = {-55, -190, 20};
    vector<double> sit_back = {-55, -160, 20};
    vector<double> null = {0, 0, 0};
  };

  struct Leg {
    vector<double> angles = {0, 0, 0};
    vector<double> legs = {0, 0, 0, 0};
  };

  class SimulationControl : public rclcpp::Node {
   public:
    SimulationControl();

    void move_to(Leg leg, vector<double> position);
   private:
    quadruped::QuadrupedKinematics kinematics;
    PredefinedPositions positions;
    const char* input = "";
    Leg temp_legs;
};
}