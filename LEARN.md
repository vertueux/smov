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

## Installing I2C
I2C is required to control microcontrollers
```bash
sudo apt install -y libi2c-dev i2c-tools
```

## Configure Ubuntu
Make sure that both I2C and SPI are enabled by default. Check the file */boot/firmware/syscfg.txt* and see if you have the following lines
```txt 
dtparam=i2c_arm=on
dtparam=spi=on
```
If not, maybe you can append them on */boot/firmware/usercfg.txt* and reboot, and hopefully, that works. If that doesn't work, maybe do `sudo apt update && sudo full-upgrade` -y and see if there are any distro updates needed.

* Refer to [this post](https://askubuntu.com/questions/1273700/enable-spi-and-i2c-on-ubuntu-20-04-raspberry-pi/1273900#1273900).

You can also add the following line to */boot/config.txt*
```bash
dtparam=i2c_arm=on
```
As well as this line to */etc/modules*
```bash
i2c-dev
```
* Refer to [this post](https://raspberrypi.stackexchange.com/questions/61905/enable-i2c-on-ubuntu-mate-raspberry-pi-3).

--- 
With `raspi-config`, you can enable i2c by navigating to *Interface Options->Advanced->I2C* and then enable it.

## Testing I2C
Now when you log in you can type the following command to see all the connected devices
```bash 
sudo i2cdetect -y 1 # Or 0, depends on the device you use.
```
## Configuring the Second I2C Port
We first have to edit */boot/config.txt* 
```bash
sudo nano /boot/config.txt
```
If we scroll to the bottom we may find the remnants of the precious peripheral (if it was added here previously)
```bash
dtoverlay=i2c-rtc,pcf8523,wakeup-source #RTC
```

We now need to add an additional device under a new bus — e.g. bus 2. We do this by adding an additional line with the following content
```bash
dtoverlay=i2c-gpio,bus=2,i2c_gpio_sda=23,i2c_gpio_scl=24
```

## Manual configuration 
When using a Raspberry Pi, you must first have dtoverlay installed for the manual configuration. Install it by copying this command 
```bash
sudo apt install libraspberrypi-bin
```

If we do not want to change any files we can tell our Raspberry Pi to load the port through a shell script with the following command. This method is easier for optional loads or prototyping. The dtoverlay the command is similar to that of the config.txt file. 

```bash
sudo dtoverlay i2c-gpio bus=2 i2c_gpio_sda=23 i2c_gpio_scl=24 
```

* Refer to [this post](https://medium.com/cemac/creating-multiple-i2c-ports-on-a-raspberry-pi-e31ce72a3eb2)

## Configuring the LCD I2C Port
To configure the LCD, you'll need to set up a new I2C port, in the same way as for configuring the second microcontroller input.

After selecting the SDA & SCL port to be configured, enter this command, modifying it as required, Here's an example: 

```bash
sudo dtoverlay i2c-gpio bus=3 i2c_gpio_sda=10 i2c_gpio_scl=9 
```
