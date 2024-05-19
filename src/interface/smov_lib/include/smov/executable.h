#ifndef EXECUTABLE_H_
#define EXECUTABLE_H_

#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include "smov_states_msgs/msg/states_servos.hpp"
#include "smov_states_msgs/msg/end_state.hpp"

namespace smov {

#define STATE_CLASS(name) void on_start();\
                          void on_loop();\
                          void on_quit();\
                          void set_name() {front_servos.state_name = #name; back_servos.state_name = #name; end_state.state_name = #name;}\
                          void delay(int time) {struct timespec ts = {0,0}; ts.tv_sec = time / 1000; ts.tv_nsec = (time % 1000) * 1000000; nanosleep(&ts, NULL);}\
                          public: void end_program() {on_quit(); end_state_publisher->publish(end_state); tcsetattr(STDIN_FILENO, TCSANOW, &old_chars); rclcpp::shutdown();}\
                          double upper_leg_length = 0.0;\
                          double lower_leg_length = 0.0;\
                          double hip_body_distance = 0.0;\
                          struct termios old_chars, new_chars;\
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
      init_reader(0);\
      RCLCPP_INFO(rclcpp::get_logger(node_name), "Press 'escape' to succesfully exit the program.");\
      auto parameters = parameters_client->get_parameters({"upper_leg_length", "lower_leg_length", "hip_body_distance"});\
      state.upper_leg_length = parameters[0].as_double();\
      state.lower_leg_length = parameters[1].as_double();\
      state.hip_body_distance = parameters[2].as_double();\
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
    void init_reader(int echo) {\
      fcntl(0, F_SETFL, O_NONBLOCK);\
      tcgetattr(0, &state.old_chars);\
      state.new_chars = state.old_chars;\
      state.new_chars.c_lflag &= ~ICANON;\
      state.new_chars.c_lflag &= echo ? ECHO : ~ECHO;\
      tcsetattr(0, TCSANOW, &state.new_chars);\
    }\
    size_t count;\
    rclcpp::TimerBase::SharedPtr timer;\
    rclcpp::TimerBase::SharedPtr quick_timer;\
  };\
  void quick_timer_callback() {\
    while (rclcpp::ok()) {\
      int c = getchar();\
      if (c == 27) {\
        RCLCPP_INFO(rclcpp::get_logger(node_name), "Quitting.");\
        state.end_program();\
      }\
    }\
  }\

#define DECLARE_STATE_NODE_CLASS(node_name, state_class, timeout)\
  CREATE_NODE_CLASS(node_name, state_class, timeout)\
  int main(int argc, char **argv)\
  {\
    rclcpp::init(argc, argv);\
    std::thread exit_thread(quick_timer_callback);\
    rclcpp::spin(std::make_shared<StateNode>());\
    exit_thread.join();\
    tcsetattr(STDIN_FILENO, TCSANOW, &state.old_chars);\
    rclcpp::shutdown();\
    return 0;\
  }\

#define DECLARE_STATE_NODE_CLASS_GET_ARGS(node_name, state_class, timeout, _argc, _argv)\
  CREATE_NODE_CLASS(node_name, state_class, timeout)\
  int main(int argc, char **argv)\
  {\
    _argc = argc;\
    _argv = argv;\
    rclcpp::init(argc, argv);\
    std::thread exit_thread(quick_timer_callback);\
    rclcpp::spin(std::make_shared<StateNode>());\
    exit_thread.join();\
    tcsetattr(STDIN_FILENO, TCSANOW, &state.old_chars);\
    rclcpp::shutdown();\
    return 0;\
  }\

} // namespace smov

#endif // EXECUTABLE_H_
