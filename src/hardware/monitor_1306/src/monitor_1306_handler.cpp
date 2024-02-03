//
// Created by ros on 2/2/24.
//

#include "monitor_1306/monitor_1306_handler.h"

namespace smov {

    Monitor1306Handler::Monitor1306Handler() : Node("monitor_1306"){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting the SSD1306 Driver.");

    }
}