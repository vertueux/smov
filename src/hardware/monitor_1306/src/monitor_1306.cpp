#include <monitor_1306/monitor_1306_handler.h>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<smov::Monitor1306Handler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
