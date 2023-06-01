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
      publisher_ = this->create_publisher<i2c_pwm_board_msgs::msg::ServoArray>("/servos_proportional", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&ServoController::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = i2c_pwm_board_msgs::msg::ServoArray();
      message.servos[0].servo = 1;
      message.servos[0].value = 0.4;
      message.servos[1].servo = 2;
      message.servos[1].value = 0.4;

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