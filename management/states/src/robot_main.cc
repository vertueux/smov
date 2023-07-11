#include <states/robot_node_handler.h>
#include <states/robot_manager.h>

int main(int argc, char * argv[]) {
  smov::RobotManager* robot = smov::RobotManager::Instance();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::RobotNodeHandle>());
  robot->on_quit();
  rclcpp::shutdown();
  return 0;
}

