<h1 align="center">
  <p align="center">
    The SMOV Project
  </p>
  <p align="center">
    <a href="https://github.com/vertueux/smov/blob/master/LICENSE"><img src="https://img.shields.io/badge/License-GPL%203.0.2-blue"/></a>
    <a href="https://discord.gg/4m2SgCmWMr"><img src="https://img.shields.io/badge/Join%20the%20Discord%20server%20for%20help%20&%20questions-Click%20here-informational"/></a>
  </p>
</h1>

SMOV is the robot I was and still working on. It is a quadruped dog robot which would have never existed without seeing ["Spot" from Boston Dynamics](https://www.bostondynamics.com/products/spot#:~:text=Spot%20is%20an%20agile%20mobile,Automate). Its main objective is to have similar capabilities to this one, but costing much less, being more modular & extensible, and having a source code accessible to all.

https://github.com/vertueux/smov/assets/81981323/b2234707-8f5c-4f4c-adbb-8d1c0cae671e

## Table of Contents

* [Overview](#overview)
* [What makes SMOV different?](#what-makes-smov-different)
* [Supported Platforms](#supported-platforms)
* [How can I get quickly started with my robot?](#how-can-i-get-quickly-started-with-my-robot)
* [Where is the documentation?](#where-is-the-documentation)
* [How can I contribute?](#how-can-i-contribute)
* [Links and References](#links-and-references)

## Overview
SMOV is a 4 legged open source robot. It is built on a modular architecture that facilitates third-party control and application execution according to your needs. You don't have to reprogram the whole robot to adapt it to your needs. Just create a simple executable using the SMOV's simple C++ programming interface.

For those who already have a robot built (such as the Spot Micro), don't worry, the SMOV has been designed to integrate with the model, and doesn't currently require you to modify or create a new one.

## What makes SMOV different?
SMOV is designed to be easily extensible. It is centralized in a single executable, the States package. This package, while running, will listen for any message sent by a third-party package and apply them to the real robot. In other words, this modular architecture facilitates robot control, and makes it easier to perform any application. 

This is achieved by some sort of API similar to the Arduino's one, but specific to robot control. Here's an example taken directly from [the robot state template](https://github.com/vertueux/smov_state):
```cpp
#include <template/template.h>

namespace smov {

void TemplateState::on_start() {
  // This will be called when the node starts running.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "What's up world ?");

  delay(2000); // Delay of 2 seconds (2000ms).

  // Putting all the servos to their center. (Min: -1.0, Max: 1.0).
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_servos.value[i] = 0.0f; 
    back_servos.value[i] = 0.0f;
  }

  // Then publishing to the States package to apply them 
  // to the real robot.
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  // If your program consists just of a function 
  // not requiring any loop, on can directly end 
  // the program by using this function:
  // end_program();
  end_program();
}

void TemplateState::on_loop() {
  // This will is every 500ms (But you 
  // can change the timeout at DECLARE_STATE_NODE_CLASS).
}

void TemplateState::on_quit() {
  // This is called when the program gets shutdown.
}

}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("smov_template_state", smov::TemplateState, 500ms)
```

So you can create your own package to control the robot according to your needs. You might even be able to publish it and share it with your colleagues and other four-legged robot creators.

## Supported Platforms
This package is designed and tested to be compatible with ROS2 Humble and ROS2 Foxy running on a [Raspberry Pi](https://www.raspberrypi.com/) with other tools like servos & micro-controllers. 
But it should also be able to run on a ESP32.
> **Note**: x86_64 system may not be supported as this projet is using a Raspberry Pi (except for simulation and other tools).

| Platform | Hardware                                                                                                                                                                                                | Software                                                       | Notes                                                                                                                                                                                                                                                                                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| Raspberry Pi   | Pi 3 Model B+<br/> Pi 4 Model B<br/> | [Ubuntu 22.04 LTS](https://ubuntu.com/download/raspberry-pi)<br/> [Ubuntu 20.04 LTS](https://ubuntu.com/download/raspberry-pi) | For now, the project is currently being only tested on the Raspberry Pi 3 Model B+, but should possibly (and surely) be available on other Raspberry Pi.  |

## How can I get quickly started with my robot?
To get started quickly and configure your robot, take a look at [QUICK_START.md](documentation/QUICK_START.md).

## Where is the documentation?
Don't worry, I am currently working on the documentation, so you'll know how to adapt your robot to work with SMOV, and how to create the robot. But right now, for an overview and simple documentation on how to configure the robot, take a look at [README.md](documentation/README.md).


## How can I contribute?
To be able to contribute, you need to take a look at [CONTRIBUTING.md](CONTRIBUTING.md) to find out the conditions.

## Links and References
* The I2C PWM Board repository: 
  * [Click here](https://github.com/vertueux/i2c_pwm_board)
* My (deprecated) quadruped kinematics repository: 
  * [Click here](https://github.com/vertueux/quadruped_kinematics)
