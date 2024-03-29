#ifndef ROBOT_STATES_H_
#define ROBOT_STATES_H_

#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>

#include "std_msgs/msg/string.hpp"

#include <states/robot_manager.h>

#include "smov_states_msgs/msg/states_servos.hpp"
#include "smov_states_msgs/msg/end_state.hpp"

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
                          void delay(int time) {struct timespec ts = {0,0}; ts.tv_sec = time / 1000; ts.tv_nsec = (time % 1000) * 1000000; nanosleep(&ts, NULL);}\
                          public: void end_program() {end_state_publisher->publish(end_state);}\
                          smov_states_msgs::msg::StatesServos front_servos;\
                          smov_states_msgs::msg::StatesServos back_servos;\
                          smov_states_msgs::msg::EndState end_state;\
                          rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr front_state_publisher;\
                          rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr back_state_publisher;\
                          rclcpp::Publisher<smov_states_msgs::msg::EndState>::SharedPtr end_state_publisher;\

#define STATE_LIBRARY_CLASS(name) smov_states_msgs::msg::StatesServos* front_servos;\
                                  smov_states_msgs::msg::StatesServos* back_servos;\
                                  void delay(int time) {struct timespec ts = {0,0}; ts.tv_sec = time / 1000; ts.tv_nsec = (time % 1000) * 1000000; nanosleep(&ts, NULL);}\
                                  rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr* front_state_publisher;\
                                  rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr* back_state_publisher;\
                                  name(smov_states_msgs::msg::StatesServos* f_servos, smov_states_msgs::msg::StatesServos* b_servos,\
                                   rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr* f_pub,\
                                   rclcpp::Publisher<smov_states_msgs::msg::StatesServos>::SharedPtr* b_pub)\
                                   : front_servos(f_servos), back_servos(b_servos),\
                                   front_state_publisher(f_pub), back_state_publisher(b_pub) { }\

#define DECLARE_STATE_NODE_CLASS(node_name, state_class, timeout)\
  using namespace std::chrono_literals;\
  state_class state;\
  struct termios old_chars, new_chars;\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.set_name();\
      init_reader(0);\
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
      tcgetattr(0, &old_chars);\
      new_chars = old_chars;\
      new_chars.c_lflag &= ~ICANON;\
      new_chars.c_lflag &= echo ? ECHO : ~ECHO;\
      tcsetattr(0, TCSANOW, &new_chars);\
    }\
    size_t count;\
    rclcpp::TimerBase::SharedPtr timer;\
    rclcpp::TimerBase::SharedPtr quick_timer;\
  };\
  void quick_timer_callback() {\
    while (rclcpp::ok()) {\
      int c = getchar();\
      if (c == 27) {\
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quitting.");\
        state.end_program();\
        rclcpp::shutdown();\
      }\
    }\
  }\
  int main(int argc, char **argv)\
  {\
    rclcpp::init(argc, argv);\
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Remember to press 'escape' to successfully exit the program.");\
    std::thread exit_thread(quick_timer_callback);\
    rclcpp::spin(std::make_shared<StateNode>());\
    state.on_quit();\
    exit_thread.join();\
    tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);\
    rclcpp::shutdown();\
    return 0;\
  }\

#define DECLARE_STATE_NODE_CLASS_GET_ARGS(node_name, state_class, timeout, _argc, _argv)\
  using namespace std::chrono_literals;\
  state_class state;\
  struct termios old_chars, new_chars;\
  void end_program() {state.end_state_publisher->publish(state.end_state);}\
  class StateNode : public rclcpp::Node{\
   public:\
    StateNode()\
    : Node(node_name), count(0) {\
      state.set_name();\
      init_reader(0);\
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
      tcgetattr(0, &old_chars);\
      new_chars = old_chars;\
      new_chars.c_lflag &= ~ICANON;\
      new_chars.c_lflag &= echo ? ECHO : ~ECHO;\
      tcsetattr(0, TCSANOW, &new_chars);\
    }\
    size_t count;\
    rclcpp::TimerBase::SharedPtr timer;\
    rclcpp::TimerBase::SharedPtr quick_timer;\
  };\
  void quick_timer_callback() {\
    while (rclcpp::ok()) {\
      int c = getchar();\
      if (c == 27) {\
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Quitting.");\
        end_program();\
        rclcpp::shutdown();\
      }\
    }\
  }\
  int main(int argc, char **argv)\
  {\
    _argc = argc;\
    _argv = argv;\
    rclcpp::init(argc, argv);\
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Remember to press 'escape' to successfully exit the program.");\
    std::thread exit_thread(quick_timer_callback);\
    rclcpp::spin(std::make_shared<StateNode>());\
    state.on_quit();\
    exit_thread.join();\
    tcsetattr(STDIN_FILENO, TCSANOW, &old_chars);\
    rclcpp::shutdown();\
    return 0;\
  }\

} // namespace smov

#endif // ROBOT_STATES_H_