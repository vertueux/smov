<h1 align="center">
  <p align="center">
    The SMOV Project
  </p>
  <p align="center">
    <a href="https://github.com/vertueux/smov/blob/master/LICENSE"><img src="https://img.shields.io/badge/License-EUPL_1.2-blue"/></a>
    <a href="https://discord.com/"><img src="https://img.shields.io/badge/Join%20the%20Discord%20server%20for%20help%20&%20questions-Click%20here-informational"/></a>
  </p>
</h1>

SMOV is the robot I was and still working on. It is a quadruped dog robot which would never have never existed without seeing ["Spot" from Boston Dynamics](https://www.bostondynamics.com/products/spot#:~:text=Spot%20is%20an%20agile%20mobile,Automate). It's main objective is to have similar capabilities to this one, but costing much less, being more modular & extensible, and having a source code accessible to all.

![Special SMOV Awakening1](https://github.com/vertueux/smov/assets/81981323/562bf6d2-6a03-461b-bf25-ed146b75184d)
![Special SMOV Awakening2](assets/special_smov_awakening.mp4)

## Table of Contents

* [Overview](#overview)
* [What makes SMOV different?](#what-makes-smov-different)
* [Context & Background](#context--background)
* [Supported Platforms](#supported-platforms)
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
  // This will called every 500ms (But you 
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
## Context & Background
> My father, for a long time, wanted to create his 4 legged robot, but not having the time to do it, he asked me to do it for him, and taking advantage of it to make the project using the last version of ROS (2). And so, here we are. But I don't intend to stop there.

I started this project at the beginning of the year 2022 in January, and since then I have spent a minimum of time on it. Unfortunately, 2022 is also the year I take my final middle school exam ("Le Brevet", in french), allowing me to go to high school. With these exams, as well as the other sub-projects and learning that I was doing externally, the project has not progressed that much. 

But I'm now back, in August, and ready to make great progress on this project.

## Supported Platforms
This package is designed and tested to be compatible with ROS2 Humble and ROS2 Foxy running on a [Raspberry Pi](https://www.raspberrypi.com/) with other tools like servos & micro-controllers. 
But it should also be able to run on a ESP32.
> **Note**: x86_64 system may not be supported as this projet is using a Raspberry Pi (except for simulation and other tools).

| Platform | Hardware                                                                                                                                                                                                | Software                                                       | Notes                                                                                                                                                                                                                                                                                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| Raspberry Pi   | Pi 3 Model B+<br/> Pi 4 Model B<br/> | [Ubuntu 22.04 LTS](https://ubuntu.com/download/raspberry-pi)<br/> [Ubuntu 20.04 LTS](https://ubuntu.com/download/raspberry-pi) | For now, the project is currently being only tested on the Raspberry Pi 3 Model B+, but should possibly (and surely) be available on other Raspberry Pi.  |

## Links and References
* The I2C PWM Board repository: 
  * [Click here](https://github.com/vertueux/i2c_pwm_board)
* My (deprecated) quadruped kinematics repository: 
  * [Click here](https://github.com/vertueux/quadruped_kinematics)
