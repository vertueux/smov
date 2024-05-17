#ifndef SMOV_LIBRARY_H_
#define SMOV_LIBRARY_H_

#include <ctime>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include "smov_states_msgs/msg/states_servos.hpp"
#include "smov_states_msgs/msg/end_state.hpp"

namespace smov {

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

}

#endif // SMOV_LIBRARY_H_
