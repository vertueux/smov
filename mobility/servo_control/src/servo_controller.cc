#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "i2c_pwm_board_msgs/msg/servo_array.hpp"
#include "i2c_pwm_board_msgs/msg/servo.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ServoController : public rclcpp::Node
{
  public:
    ServoController()
    : Node("servo_controller"), count_(0)
    {
      RCLCPP_INFO(this->get_logger(), "Yes");
      publisher_ = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("servos_proportional", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&ServoController::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto servo1 = i2c_pwm_board_msgs::msg::Servo();
      servo1.servo = 1;
      servo1.value = -1.0;
      auto message = i2c_pwm_board_msgs::msg::ServoArray();
      message.servos.push_back(servo1);
      RCLCPP_INFO(this->get_logger(), "Publishing");
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<i2c_pwm_board_msgs::msg::ServoArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoController>());
  rclcpp::shutdown();
  return 0;
}
