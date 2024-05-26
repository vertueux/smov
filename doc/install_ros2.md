
# Install ROS2

Here's a tutorial showing how to install ROS2 Humble from the command line on a machine running Ubuntu, from the [official site](https://docs.ros.org/en/humble/Installation.html).
If you'd rather install the Foxy version, you can do so by clicking [here](https://docs.ros.org/en/foxy/Installation.html).

If you already have a ROS2 installation, you can skip straight to [the next step](install_libraries.md).

> **Note**: All the text shown below is taken directly from [this link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Set locale

Make sure you have a locale which supports ```UTF-8```. If you are in a minimal environment (such as a docker container), the locale may be something minimal like POSIX. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

## Setup Sources

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

## Install ROS 2 packages

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

## Environment setup

Set up your environment by sourcing the following file.

```bash
source /opt/ros/humble/setup.bash
```

## Installing colcon

```bash
sudo apt install python3-colcon-common-extensions
```

**Next step**: [Install libraries](install_libraries.md)
