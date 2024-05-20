#ifndef EXECUTABLE_H_
#define EXECUTABLE_H_

#include <ctime>
#include <unistd.h>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include "smov_states_msgs/msg/states_servos.hpp"
#include "smov_states_msgs/msg/end_state.hpp"

#define UNUSED(expr) do { (void)(expr); } while (0);

namespace smov {

#define STATE_CLASS(name) void on_start();\
                          void on_loop();\
                          void on_quit();\
                          void set_name() {front_servos.state_name = #name; back_servos.state_name = #name; end_state.state_name = #name;}\
                          void delay(int time) {struct timespec ts = {0,0}; ts.tv_sec = time / 1000; ts.tv_nsec = (time % 1000) * 1000000; nanosleep(&ts, NULL);}\
                          public: void end_program() {on_quit(); end_state_publisher->publish(end_state); RCLCPP_INFO(rclcpp::get_logger(#name), "Quitting."); rclcpp::shutdown(); abort();}\
                          double upper_leg_length = 0.0;\
                          double lower_leg_length = 0.0;\
                          double hip_body_distance = 0.0;\
                          smov_states_msgs::msg::StatesServos front_servos;\
                          smov_states_msgs::msg::StatesServos back_servos;\
                          smov_states_msgs::msg::EndState end_state;\
                          rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr front_state_publisher;\
                          rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr back_state_publisher;\
                          rclcpp::Publisher<smov_states_msgs::msg::EndState>::SharedPtr end_state_publisher;\

#define CREATE_NODE_CLASS(node_name, state_class, timeout)\
  using namespace std::chrono_literals;\
  state_class state;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "smov_states");\
      while (!parameters_client->wait_for_service(std::chrono::seconds(1))) {\
        if (!rclcpp::ok()) {\
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the SMOV States service. Exiting.");\
          return;\
        }\
        RCLCPP_INFO(this->get_logger(), "SMOV States service not available, waiting again...");\
      }\
      auto parameters = parameters_client->get_parameters({"upper_leg_length", "lower_leg_length", "hip_body_distance"});\
      state.upper_leg_length = parameters[0].as_double();\
      state.lower_leg_length = parameters[1].as_double();\
      state.hip_body_distance = parameters[2].as_double();\
      signal(SIGINT, sigint_handler);\
      state.set_name();\
      state.front_state_publisher =\
        this->create_publisher<smov_states_msgs::msg::StatesServos>("front_proportional_servos", 50);\
      state.back_state_publisher =\
        this->create_publisher<smov_states_msgs::msg::StatesServos>("back_proportional_servos", 50);\
      state.end_state_publisher =\
        this->create_publisher<smov_states_msgs::msg::EndState>("end_state", 1);\
      timer = this->create_wall_timer(timeout, std::bind(&StateNode::timer_callback, this));\
      state.on_start();\
    }\
   private:\
    void timer_callback() {\
      state.on_loop();\
    }\
    static void sigint_handler(int signum) {\
      UNUSED(signum)\
      state.end_program();\
    }\
    size_t count;\
    rclcpp::TimerBase::SharedPtr timer;\
    rclcpp::TimerBase::SharedPtr quick_timer;\
  };\

#define DECLARE_STATE_NODE_CLASS(node_name, state_class, timeout)\
  CREATE_NODE_CLASS(node_name, state_class, timeout)\
  int main(int argc, char **argv)\
  {\
    rclcpp::init(argc, argv);\
    rclcpp::spin(std::make_shared<StateNode>());\
    return 0;\
  }\

#define DECLARE_STATE_NODE_CLASS_GET_ARGS(node_name, state_class, timeout, _argc, _argv)\
  CREATE_NODE_CLASS(node_name, state_class, timeout)\
  int main(int argc, char **argv)\
  {\
    _argc = argc;\
    _argv = argv;\
    rclcpp::init(argc, argv);\
    rclcpp::spin(std::make_shared<StateNode>());\
    return 0;\
  }\

} // namespace smov

#endif // EXECUTABLE_H_
