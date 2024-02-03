#include <states/robot_node_handler.h>
#include <states/robot_manager.h>


int main(int argc, char * argv[]) {
  auto node = std::make_shared<smov::RobotNodeHandle>();
  rclcpp::init(argc, argv);
  rclcpp::spin(node);
  node->stop_servos();
  rclcpp::shutdown();
  return 0;
}

