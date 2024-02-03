//
// Created by ros on 2/2/24.
//

#ifndef SMOV_MONITOR_1306_HANDLER_H
#define SMOV_MONITOR_1306_HANDLER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <rclcpp/rclcpp.hpp>
#include "monitor_msgs/msg/display_text.hpp"

namespace smov {

    class Monitor1306Handler : public rclcpp::Node {

    public:
        Monitor1306Handler();

    private:
        void setup_subscriptions();

        rclcpp::Subscription<monitor_msgs::msg::DisplayText>::SharedPtr display_text_sub;

        int _display_i2c_handle;

        enum SSD1306Registers {
            // Registers / etc.
            __SET_CONTRAST = 0x81,
            __SET_ENTIRE_ON = 0xA4,
            __SET_NORM_INV = 0xA6,
            __SET_DISP = 0xAE,
            __SET_MEM_ADDR = 0x20,
            __SET_COL_ADDR = 0x21,
            __SET_PAGE_ADDR = 0x22,
            __SET_DISP_START_LINE = 0x40,
            __SET_SEG_REMAP = 0xA0,
            __SET_MUX_RATIO = 0xA8,
            __SET_IREF_SELECT = 0xAD,
            __SET_COM_OUT_DIR = 0xC0,
            __SET_DISP_OFFSET = 0xD3,
            __SET_COM_PIN_CFG = 0xDA,
            __SET_DISP_CLK_DIV = 0xD5,
            __SET_PRECHARGE = 0xD9,
            __SET_VCOM_DESEL = 0xDB,
            __SET_CHARGE_PUMP = 0x8D,

        };


    };
}

#endif //SMOV_MONITOR_1306_HANDLER_H
