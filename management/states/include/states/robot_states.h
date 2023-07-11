#pragma once

#include <string>

#include <states/robot_manager.h>

namespace smov {

#define STATE_CLASS(name) void on_start();\
                          void on_loop();\
                          enum RobotParts {LEFT_BODY, RIGHT_BODY, LEFT_BICEPS, RIGHT_BICEPS, LEFT_LEG, RIGHT_LEG};\
                          states_msgs::msg::StatesServos front_values;states_msgs::msg::StatesServos back_values;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr front_state_publisher;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr back_state_publisher;\
                          rclcpp::TimerBase::SharedPtr timer;

#define STATE_NODE_CLASS(node_name,state_class,timeout)\
  using namespace std::chrono_literals;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.on_start();\
      state.front_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("front_proportional_servos", 1);\
      state.back_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("back_proportional_servos", 1);\
      state.timer = this->create_wall_timer(timeout, std::bind(&StateNode::timer_callback, this));\
    }\
   private:\
    void timer_callback() {\
      state.on_loop();\
      state.front_state_publisher->publish(state.front_values);\
      state.back_state_publisher->publish(state.back_values);\
  }\
  state_class state;\
  size_t count;\
  };\
  int main(int argc, char **argv)\
  {\
    rclcpp::init(argc, argv);\
    rclcpp::spin(std::make_shared<StateNode>());\
    rclcpp::shutdown();\
    return 0;\
  }\

} // namespace smov
