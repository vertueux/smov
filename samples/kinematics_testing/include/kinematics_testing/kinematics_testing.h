#pragma once

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>

#include <states/robot_states.h>
#include <states_msgs/msg/states_servos.hpp>

namespace smov {

class KinematicsTesting {
 public:
  STATE_CLASS("Kinematics Testing")
};

} // namespace smov
