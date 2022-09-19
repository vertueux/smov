#include "motion/smov_motion_command.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::MotionControl>());
  rclcpp::shutdown();
}