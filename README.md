<h1 align="left">
  <p align="left">
    The SMOV Project
  </p>
  <p align="left>
    <a href="https://github.com/vertueux/spot_micro2/blob/master/LICENSE.md"><img src="https://img.shields.io/badge/license-MIT-important"/></a>
    <a href="https://github.com/vertueux/"><img src="https://img.shields.io/badge/Contact%20me%20on%20Discord-now%239470-informational"/></a>
  </p>
</h1>

SMOV is the robot I was and still working on. It is a quadruped dog robot which would never have never existed without having see ["Spot" from Boston Dynamics](https://www.bostondynamics.com/products/spot#:~:text=Spot%20is%20an%20agile%20mobile,Automate). Its main objective is to have similar capabilities to this one, but costing much less and having a source code accessible to all.

# Table of contents

* [Overview](#overview)
* [Context & Background](#the-story)
* [Getting Started](#getting-started)
    * [Installing ROS2 Humble](#installing-ros2-humble)
        * [Set locale](#set-locale)
        * [Setup Sources](#setup-sources)
        * [Install ROS2 Packages](#install-ros-2-packages)
        * [Environment Setup](#environment-setup)
            * [Sourcing the setup script](#sourcing-the-setup-script)
        * [Installing colcon](#installing-colcon)
    * [General Instructions](#general-instructions)
* [Links and References](#links-and-references)

# Overview
SMOV is a 4 legged open source robot. It is (theoretically for now) able to move in an environment, moving in the same way as a dog. This project is not yet in a stable and fixed version, and is still under development.


# History & Background
My father, for a long time, wanted to create his 4 legged robot, but not having the time to do it, he asked me to do it for him, and taking advantage of it to make the project using the last version of ROS (2). And so, here we are. But I don't intend to stop there.

I started this project at the beginning of the year 2022 in January, and since then I have spent a minimum of time on it. Unfortunately, 2022 is also the year I take my final middle school exam ("Le Brevet", in french), allowing me to go to high school. With these exams, as well as the other sub-projects and learning that I was doing externally, the project has not progressed that much. But I'm now back, in August, after having moved around a lot and having finished the rest (except for [Opticus](https://github.com/vertueux/opticus/)), and ready to make great progress on this project.

# Getting Started
Here are all the instructions to run the robot simulation on your computer. Note that if you already have ROS2 Humble installed on your computer, you can skip the "Installing ROS2 Humble" part.

## Installing ROS2 Humble

### Set locale
Make sure you have a locale which supports ```UTF-8```. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### Setup Sources
You will need to add the ROS 2 apt repository to your system. First, make sure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled by checking the output of this command.
```bash
apt-cache policy | grep universe
```
This should output a line like the one below:
```bash
500 http://us.archive.ubuntu.com/ubuntu jammy/universe amd64 Packages
    release v=22.04,o=Ubuntu,a=jammy,n=jammy,l=Ubuntu,c=universe,b=amd64
```
If you don’t see an output line like the one above, then enable the Universe repository with these instructions.
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Now add the ROS 2 apt repository to your system. First authorize our GPG key with apt.
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
Then add the repository to your sources list.
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 packages
Update your apt repository caches after setting up the repositories.
```bash
sudo apt update
```
ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.
```bash
sudo apt upgrade
```
Desktop Install (Recommended): ROS, RViz, demos, tutorials.
```bash
sudo apt install ros-humble-desktop
```
ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.
```bash
sudo apt install ros-humble-ros-base
```

### Environment setup
#### Sourcing the setup script
Set up your environment by sourcing the following file.
```bash
source /opt/ros/humble/setup.bash
```

### Installing colcon
```bash
sudo apt install python3-colcon-common-extensions
```

## General Instructions
Install all SMOV packages and sub-modules. More instructions will come after the simulation packages arrive.
```bash
git clone --recursive https://github.com/vertueux/smov.git
```

# Links and References
* The Spot Micro AI community is hosted at: https://gitlab.com/custom_robots/spotmicroai
* [@mike4192](https://github.com/mike4192)'s README.md & Spot Micro for inspiration: https://github.com/mike4192/spotMicro 
* Spot micro URDF model inspired from Florian Wilk's repository: 
  * https://gitlab.com/public-open-source/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/urdf
* My quadruped kinematics for for kinematics calculation: 
  * https://github.com/vertueux/quadruped_kinematics