#pragma once

#include "std_msgs/msg/string.hpp"

#include <states/robot_manager.h>

namespace smov {

#define STATE_CLASS(name) void on_start();\
                          void on_loop();\
                          void on_quit();\
                          void set_name() {state_name.data = name;}\
                          enum RobotParts {LEFT_BODY, RIGHT_BODY, LEFT_BICEPS, RIGHT_BICEPS, LEFT_LEG, RIGHT_LEG};\
                          states_msgs::msg::StatesServos front_servos;states_msgs::msg::StatesServos back_servos;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr front_state_publisher;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr back_state_publisher;\
                          rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher;\
                          std_msgs::msg::String state_name;\
                          rclcpp::TimerBase::SharedPtr timer;

#define DECLARE_STATE_NODE_CLASS(node_name,state_class,timeout)\
  using namespace std::chrono_literals;\
  state_class state;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.set_name();\
      state.front_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("front_proportional_servos", 1);\
      state.back_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("back_proportional_servos", 1);\
      state.state_publisher =\
        this->create_publisher<std_msgs::msg::String>("last_current_state", 1);\
      state.timer = this->create_wall_timer(timeout, std::bind(&StateNode::timer_callback, this));\
      state.on_start();\
    }\
   private:\
    void timer_callback() {\
      state.on_loop();\
      state.front_state_publisher->publish(state.front_servos);\
      state.back_state_publisher->publish(state.back_servos);\
      state.state_publisher->publish(state.state_name);\
  }\
  size_t count;\
  };\
  int main(int argc, char **argv)\
  {\
    rclcpp::init(argc, argv);\
    rclcpp::spin(std::make_shared<StateNode>());\
    state.on_quit();\
    rclcpp::shutdown();\
    return 0;\
  }\

#define STATE_CLASS_INCLUDE_PARAMS(name) void on_start();\
                          void on_loop();\
                          void on_quit();\
                          void set_name() {state_name.data = name;}\
                          enum RobotParts {LEFT_BODY, RIGHT_BODY, LEFT_BICEPS, RIGHT_BICEPS, LEFT_LEG, RIGHT_LEG};\
                          states_msgs::msg::StatesServos front_servos;states_msgs::msg::StatesServos back_servos;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr front_state_publisher;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr back_state_publisher;\
                          rclcpp::TimerBase::SharedPtr timer;\
                          rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher;\
                          std_msgs::msg::String state_name;\
                          std::array<std::string, 12> servo_name = {"AVCG", "AVCD", "AVBG",\
                                                                    "AVBD", "AVJG", "AVJD",\
                                                                    "ARCG", "ARCD", "ARBG",\
                                                                    "ARBD", "ARJG", "ARJD"};\
                          std::vector<int> front_servos_data;\
                          std::vector<int> back_servos_data;

#define DECLARE_STATE_NODE_CLASS_INCLUDE_PARAMS(node_name,state_class,timeout)\
  using namespace std::chrono_literals;\
  state_class state;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.set_name();\
      for (size_t j = 0; j < state.servo_name.size(); j++)\
        this->declare_parameter(state.servo_name[j], 0);\
      for (int i = 0; i < SERVO_MAX_SIZE; i++) {\
        state.front_servos_data.push_back(this->get_parameter(state.servo_name[i]).as_int());\
        state.back_servos_data.push_back(this->get_parameter(state.servo_name[i + SERVO_MAX_SIZE]).as_int());\
      }\
      state.front_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("front_proportional_servos", 1);\
      state.back_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("back_proportional_servos", 1);\
      state.state_publisher =\
        this->create_publisher<std_msgs::msg::String>("last_current_state", 1);\
      state.timer = this->create_wall_timer(timeout, std::bind(&StateNode::timer_callback, this));\
      state.on_start();\
    }\
   private:\
    void timer_callback() {\
      state.on_loop();\
      state.front_state_publisher->publish(state.front_servos);\
      state.back_state_publisher->publish(state.back_servos);\
      state.state_publisher->publish(state.state_name);\
      for (int i = 0; i < SERVO_MAX_SIZE; i++) {\
        state.front_servos_data[i] = this->get_parameter(state.servo_name[i]).as_int();\
        state.back_servos_data[i] = this->get_parameter(state.servo_name[i + SERVO_MAX_SIZE]).as_int();\
      }\
  }\
  size_t count;\
  };\
  int main(int argc, char **argv)\
  {\
    rclcpp::init(argc, argv);\
    rclcpp::spin(std::make_shared<StateNode>());\
    state.on_quit();\
    rclcpp::shutdown();\
    return 0;\
  }\

} // namespace smov
