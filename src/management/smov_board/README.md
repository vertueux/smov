# The (Dual) Board package

> Click [here](https://github.com/vertueux/i2c_pwm_board) to view the original project (with more information about the
> installation).

This is a **ROS2** node for controlling two PWM boards based on the PCA9685 chip with an I2C interface. Primary use is
for controlling RC Servos and DC motors via PWM. This is based
on [ros-i2cpwmboard](https://gitlab.com/bradanlane/ros-i2c_pwmboard) *(OUTDATED)* with updates to make it work on ROS2.

The `src/` directory contains two C++ files: the front and back (rear) microcontrollers. Each has an address `i2c-n`
with *n* between 0 and 4 (In my case). For example, I personally defined the front microcontroller at address `i2c-1`
and the back microcontroller at `i2c-4` (See the instructions for configuring *GPIO23*-*24* with i2c, if you wish to do
the same).

Note that both microcontrollers default to address `0x40`, but this may not be the case for you. To check the
microcontroller address, you can run the command:

```bash
sudo i2cdetect -y 1 # or 2,3,4,etc..
``` 

Note that, for example, normally all Raspberry Pi devices have a default i2c address of `i2c-1`, but the Raspberry Pi
Zero has it set to `i2c-0`.

Once you've found the address, you can navigate to one of the two C++ files for the two separate microcontrollers (in
the `src/` directory) and change the base address (on line 56) from `#define _BASE_ADDR 0x40` to your new address,
e.g. `#define _BASE_ADDR 0x27`.

## Installation

You need to have ROS2 installed and theses packages provided by the default desktop installation below:

* **rclcpp, std_msgs, std_srvs, geometry_msgs**.
* **rosidl_default_generators, rosidl_default_runtime**.
* Have `python3-colcon-common-extensions` installed.
* Have `libi2c-dev` and `i2c-tools` installed.
* Have the `board_msgs`.
* Have the [xmlrpcpp](https://github.com/bpwilcox/xmlrpcpp) package (in the `hardware/` directory).

## To run it

To launch the project and the executables after compilation, you'll need to enter the two commands on two separate
terminals (for the moment, there's no `.launch` file).
Firstly, to launch the microcontroller before:

```bash
ros2 run smov_board controller 1
```

Then, in a separate terminal, paste:

```bash
ros2 run smov_board controller 4 # Or your specified bus.
```

## Examples

Theses are two examples on how to publish messages through the command line on both boards.

### On the front board

```bash
# Configuring right servos on the front board.
ros2 service call /front_config_servos board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 333, range: 100, direction: 1},{servo: 15, center: 333, range: 100, direction: 1},{servo: 14, center: 333, range: 100, direction: 1}]"

# Configuring left servos on the front board.
ros2 service call /front_config_servos board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 333, range: 100, direction: -1},{servo: 2, center: 333, range: 100, direction: -1},{servo: 3, center: 333, range: 100, direction: -1}]"

# Drive two servos forward at 40% of maximum speed on the front board.
ros2 topic pub -1 /front_servos_proportional board_msgs/msg/ServoArray "{servos:[{servo: 1, value: 0.40}, {servo: 2, value: 0.40}]}"
```

### On the back board

```bash
# Configuring right servos on the back board.
ros2 service call /back_config_servos board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 333, range: 100, direction: -1},{servo: 15, center: 333, range: 100, direction: -1},{servo: 14, center: 333, range: 100, direction: -1}]"

# Configuring left servos on the back board.
ros2 service call /back_config_servos board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 333, range: 100, direction: 1},{servo: 2, center: 333, range: 100, direction: 1},{servo: 3, center: 333, range: 100, direction: 1}]"

# Drive two servos forward at 40% of maximum speed on the back board.
ros2 topic pub -1 /back_servos_proportional board_msgs/msg/ServoArray "{servos:[{servo: 1, value: 0.40}, {servo: 2, value: 0.40}]}"
```

> More documentation at the original page of the project (for
> ROS1). [Click here](https://github.com/mentor-dyun/ros-i2cpwmboard/tree/master/doc)
> or [here](https://gitlab.com/fmrico/ros-i2cpwmboard/-/tree/master/doc).
