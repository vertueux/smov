#pragma once

#include <time.h>   
#include <unistd.h>

#include "std_msgs/msg/string.hpp"

#include <states/robot_manager.h>

#include "states_msgs/msg/states_servos.hpp"
#include "states_msgs/msg/end_state.hpp"

namespace smov {

enum MicroController {
  FRONT = 0,
  BACK = 1
};

enum RobotParts {
  LEFT_BODY, 
  RIGHT_BODY, 
  LEFT_BICEPS, 
  RIGHT_BICEPS, 
  LEFT_LEG, 
  RIGHT_LEG
};

#define STATE_CLASS(name) void on_start();\
                          void on_loop();\
                          void on_quit();\
                          void set_name() {front_servos.state_name = name; back_servos.state_name = name; end_state.state_name = name;}\
                          void delay(int time) {struct timespec ts; ts.tv_sec = time / 1000; ts.tv_nsec = (time % 1000) * 1000000; nanosleep(&ts, NULL);}\
                          states_msgs::msg::StatesServos front_servos;\
                          states_msgs::msg::StatesServos back_servos;\
                          states_msgs::msg::EndState end_state;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr front_state_publisher;\
                          rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr back_state_publisher;\
                          rclcpp::Publisher<states_msgs::msg::EndState>::SharedPtr end_state_publisher;\
                          rclcpp::TimerBase::SharedPtr timer;\

#define STATE_LIBRARY_CLASS(name) states_msgs::msg::StatesServos* front_servos;\
                                  states_msgs::msg::StatesServos* back_servos;\
                                  void delay(int time) {struct timespec ts; ts.tv_sec = time / 1000; ts.tv_nsec = (time % 1000) * 1000000; nanosleep(&ts, NULL);}\
                                  rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* front_state_publisher;\
                                  rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* back_state_publisher;\
                                  name(states_msgs::msg::StatesServos* f_servos, states_msgs::msg::StatesServos* b_servos,\
                                   rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* f_pub,\
                                   rclcpp::Publisher<states_msgs::msg::StatesServos>::SharedPtr* b_pub)\
                                   : front_servos(f_servos), back_servos(b_servos),\
                                   front_state_publisher(f_pub), back_state_publisher(b_pub) { }\

#define DECLARE_STATE_NODE_CLASS(node_name,state_class,timeout)\
  using namespace std::chrono_literals;\
  state_class state;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.set_name();\
      state.front_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("front_proportional_servos", 50);\
      state.back_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("back_proportional_servos", 50);\
      state.end_state_publisher =\
        this->create_publisher<states_msgs::msg::EndState>("end_state", 1);\
      state.timer = this->create_wall_timer(timeout, std::bind(&StateNode::timer_callback, this));\
      rclcpp::on_shutdown(std::bind(&StateNode::end_program, this));\
      state.on_start();\
    }\
   private:\
    void end_program() {state.end_state_publisher->publish(state.end_state);}\
    void timer_callback() {\
      state.on_loop();\
      /*state.front_state_publisher->publish(state.front_servos);*/\
      /*state.back_state_publisher->publish(state.back_servos);*/\
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

#define DECLARE_STATE_NODE_CLASS_GET_ARGS(node_name,state_class,timeout, _argc, _argv)\
  using namespace std::chrono_literals;\
  state_class state;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.set_name();\
      state.front_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("front_proportional_servos", 50);\
      state.back_state_publisher =\
        this->create_publisher<states_msgs::msg::StatesServos>("back_proportional_servos", 50);\
      state.state_publisher =\
        this->create_publisher<std_msgs::msg::String>("last_state", 1);\
      state.timer = this->create_wall_timer(timeout, std::bind(&StateNode::timer_callback, this));\
      state.on_start();\
    }\
   private:\
    void timer_callback() {\
      state.on_loop();\
      /*state.front_state_publisher->publish(state.front_servos);*/\
      /*state.back_state_publisher->publish(state.back_servos);*/\
      state.state_publisher->publish(state.state_name);\
  }\
  size_t count;\
  };\
  int main(int argc, char **argv)\
  {\
    _argc = argc;\
    _argv = argv;\
    rclcpp::init(argc, argv);\
    rclcpp::spin(std::make_shared<StateNode>());\
    state.on_quit();\
    rclcpp::shutdown();\
    return 0;\
  }\

} // namespace smov
