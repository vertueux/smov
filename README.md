<h1 align="center">
  <p align="center">
    The SMOV Project
  </p>
  <p align="center">
    <a href="https://github.com/vertueux/smov/blob/master/LICENSE"><img src="https://img.shields.io/badge/License-EUPL_1.2-blue"/></a>
    <a href="https://discord.com/"><img src="https://img.shields.io/badge/Join%20the%20Discord%20server%20-Click%20here-informational"/></a>
  </p>
</h1>

SMOV is the robot I was and still working on. It is a quadruped dog robot which would never have never existed without seeing ["Spot" from Boston Dynamics](https://www.bostondynamics.com/products/spot#:~:text=Spot%20is%20an%20agile%20mobile,Automate). It's main objective is to have similar capabilities to this one, but costing much less and having a source code accessible to all.

## Table of Contents

* [Overview](#overview)
* [Context & Background](#context--background)
* [Supported Platforms](#supported-platforms)
* [Links and References](#links-and-references)

## Overview
SMOV is a 4 legged open source robot. It is (theoretically for now) able to move in an environment, moving in the same way as a dog. This project is not yet in a stable and fixed version, and is still under development.


## Context & Background
> My father, for a long time, wanted to create his 4 legged robot, but not having the time to do it, he asked me to do it for him, and taking advantage of it to make the project using the last version of ROS (2). And so, here we are. But I don't intend to stop there.

I started this project at the beginning of the year 2022 in January, and since then I have spent a minimum of time on it. Unfortunately, 2022 is also the year I take my final middle school exam ("Le Brevet", in french), allowing me to go to high school. With these exams, as well as the other sub-projects and learning that I was doing externally, the project has not progressed that much. But I'm now back, in August, after having moved around a lot and having finished the rest (except for [Opticus](https://github.com/vertueux/opticus/)), and ready to make great progress on this project.

## Supported Platforms
This package is designed and tested to be compatible with ROS2 Humble and ROS2 Foxy running on a [Raspberry Pi](https://www.raspberrypi.com/) with other tools like servos & micro-controllers.
> **Note**: x86_64 system may not be supported as this projet is using a Raspberry Pi (except for simulation and other tools).

| Platform | Hardware                                                                                                                                                                                                | Software                                                       | Notes                                                                                                                                                                                                                                                                                                                                                       |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| Raspberry Pi   | Pi 3 Model B+<br/> Pi 4 Model B<br/> | [Ubuntu 22.04 LTS](https://ubuntu.com/download/raspberry-pi)<br/> [Ubuntu 20.04 LTS](https://ubuntu.com/download/raspberry-pi) | For now, the project is currently being only tested on the Raspberry Pi 3 Model B+, but should possibly (and surely) be available on other Raspberry Pi.  |

## Links and References
* The I2C PWM Board repository 
  * [Click here](https://github.com/vertueux/i2c_pwm_board)
* My quadruped kinematics repository 
  * [Click here](https://github.com/vertueux/quadruped_kinematics)
