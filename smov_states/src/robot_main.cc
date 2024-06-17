#include "robot.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::RobotNodeHandle>());
  rclcpp::shutdown();
  return 0;
}

