# The SMOV Project

![SMOV Discord](https://img.shields.io/badge/Discord-server?style=social&logo=discord&label=SMOV&link=https%3A%2F%2Fdiscord.gg%2F4m2SgCmWMr)

## Overview

This repository is where the base code for a 4 legged robot dog is publicly stored. To access other repositories related to the project, take a look at the [links & references](#links-and-references). SMOV is a quadruped dog robot which would have never existed without seeing ["Spot" by Boston Dynamics](https://www.bostondynamics.com/products/spot#:~:text=Spot%20is%20an%20agile%20mobile,Automate). Its main objective is to have similar capabilities to this one, but costing much less, being more modular & extensible, and having a source code accessible to all.

The SMOV project can be seen as a starting point for entering the Spot Micro environment using [ROS2](https://www.ros.org/). This is a sort of layer that allows developers to build on top of it, as most packages only deal with controlling the board, LCD and servos, leaving a small SDK to allow the user to do what they want with their robot (motion, etc...).

Join the [Discord server](https://discord.com/invite/4m2SgCmWMr) to stay tuned.

https://github.com/vertueux/smov/assets/81981323/6fc0fd70-c529-4db6-8a65-2f87200d1d53

## Features

Control of the robot is centralised in a single executable, the States package. This package, when running, listens to any message sent by a third-party package and applies it to the actual robot. In other words, this modular architecture makes it easy to control the robot and run any application.

The user can easily create another package, use the minimal SDK to control the servos through another executable, and choose what he wants his robot to do, without having to worry about numerical values, LCD control, etc...
You can create your own package to control the robot according to your needs. You might even be able to publish it and share it with your colleagues and other four-legged robot creators.

You can take a look at [the example](#example) and [demos](https://github.com/vertueux/smov_demos) to get a better idea.

## Documentation

You can find the SMOV documentation in the [doc/](doc/) subdirectory.

Check out [doc/README.md](doc/README.md) for a quick overview.

The documentation is divided into several sections:

* [Install ROS2](doc/install_ros2.md)
* [Install libraries](doc/install_libraries.md)
* [Configure ports](doc/configure_ports.md)
* [Build the project](doc/build_the_project.md)
* [Calibrate servos](doc/calibrate_servos.md)
* [Build a Spot Micro](doc/build_a_spot_micro.md)
* [Test demos](doc/test_demos.md)
* [Create your own State package](doc/create_your_own_state_package.md)

## Compatibility

This package is designed and tested to be compatible with ROS2 Humble and ROS2 Foxy running on a [Raspberry Pi](https://www.raspberrypi.com/). However, the project should be able to run on any platform that ROS2 supports.
> **Note**: x86_64 systems may not be supported as this project has been designed to run on a Raspberry Pi.

| Platform | Hardware                                                                                                                                                                                                | Software                                                       | Notes                                                                                                                                                                                                                                                                                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| Raspberry Pi   | Pi 3 Model B+<br/> Pi 4 Model B<br/> | [Ubuntu 22.04 LTS](https://ubuntu.com/download/raspberry-pi)<br/> [Ubuntu 20.04 LTS](https://ubuntu.com/download/raspberry-pi) | For the time being, the project is only being tested on the Raspberry Pi 3 Model B+ and the Raspberry Pi 4 Model B, but it should be possible to run it on other Raspberry Pis. |

## Example

Here's a simple example showing how the minimal SDK work, inspired by the [State template](https://github.com/vertueux/smov_state):

```cpp
#include <smov/base.h>
#include <template/template.h>

void State::on_start() {
  smov::delay(2000); // Delay of 2 seconds (2000ms).

  // Set all servos to 60° angle.
  for (int i = 0; i < SERVO_MAX_SIZE; i++) {
    front_servos.value[i] = 60.0f; 
    back_servos.value[i] = 60.0f;
  }

  // Then publishing to the States package to apply them to the real robot.
  front_state_publisher->publish(front_servos);
  back_state_publisher->publish(back_servos);

  // You can end the program manually.
  end_program();
}
// This is called every 500ms (But you can change the timeout at DECLARE_STATE_NODE_CLASS).
void State::on_loop() {}

// This is called when the program gets shutdown.
void State::on_quit() {}

// This macro creates the node and the main() input, which spins the node.
DECLARE_STATE_NODE_CLASS("template_state", State, 500ms)
```

## Contributing

There are many ways in which you can participate in this project, for example:

* Submit bugs and feature requests
* Review source code changes
* Review the [documentation](doc/README.md) and make pull requests for anything from typos to additional and new content

To be able to contribute, you need to take a look at [CONTRIBUTING.md](CONTRIBUTING.md) to find out the conditions.

## Code of Conduct

This project has adopted a Code of Conduct adapted from the [Contributor Covenant](https://www.contributor-covenant.org) to maintain cohesion. Please read the [full text](CODE_OF_CONDUCT.md) so that you can understand what actions will and will not be tolerated.

## Links and References

* The I²C PWM Board repository:
  * [Click here](https://github.com/vertueux/i2c_pwm_board)
* SMOV Demos repository:
  * [Click here](https://github.com/vertueux/smov_demos)
* SMOV State Template repository:
  * [Click here](https://github.com/vertueux/smov_state)
* Spot Micro AI's website:
  * [Click here](https://spotmicroai.readthedocs.io/)
