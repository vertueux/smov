#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "front_i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "front_i2c_pwm_board_msgs/msg/servo.hpp"

using namespace std::chrono_literals;

namespace smov {

class SMOVGoDown : public rclcpp::Node {
  public:
    SMOVGoDown()
    : Node("smov_go_down"), count(0) {
      RCLCPP_INFO(this->get_logger(), "the SMOV has awakened!");
      publisher = this->create_publisher<front_i2c_pwm_board_msgs::msg::ServoArray>("servos_absolute", 10);
      timer = this->create_wall_timer(500ms, std::bind(&SMOVGoDown::call, this));
    }

  private:
    void call() {
      // Setting up the first servo on port 0.
      auto first_servo = front_i2c_pwm_board_msgs::msg::Servo();
      first_servo.servo = 1;
      first_servo.value = 100;

      // Setting up the second servo on port 15.
      auto second_servo = front_i2c_pwm_board_msgs::msg::Servo();
      second_servo.servo = 16;
      second_servo.value = 548;

      auto message = front_i2c_pwm_board_msgs::msg::ServoArray();

      // Pushing the servos to the array.
      message.servos.push_back(first_servo);
      message.servos.push_back(second_servo);

      publisher->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<front_i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr publisher;
    size_t count;
};

}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smov::SMOVGoDown>());
  rclcpp::shutdown();
  return 0;
}
