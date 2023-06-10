> Click [here](https://github.com/vertueux/i2c_pwm_board) to view the original project (with more information about the installation).

**ROS2** node for controlling PWM boards based on the PCA9685 chip with an I2C interface. Primary use is for controlling RC Servos and DC motors via PWM. This is based on [ros-i2cpwmboard](https://gitlab.com/bradanlane/ros-i2c_pwmboard) *(OUTDATED)* with updates to make it work on ROS2.

# Examples
Theses are two examples on how to publish messages through the command line on both boards.
## On the front board
```bash
# Configuring right servos on the front board.
ros2 service call /config_servos front_board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 333, range: 100, direction: 1},{servo: 15, center: 333, range: 100, direction: 1},{servo: 14, center: 333, range: 100, direction: 1}]"

# Configuring left servos on the front board.
ros2 service call /config_servos front_board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 333, range: 100, direction: -1},{servo: 2, center: 333, range: 100, direction: -1},{servo: 3, center: 333, range: 100, direction: -1}]"

# Drive two servos forward at 40% of maximum speed on the front board.
ros2 topic pub -1 /servos_proportional front_board_msgs/msg/ServoArray "{servos:[{servo: 1, value: 0.40}, {servo: 2, value: 0.40}]}"
```

## On the back board
```bash
# Configuring right servos on the back board.
ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 16, center: 333, range: 100, direction: 1},{servo: 15, center: 333, range: 100, direction: 1},{servo: 14, center: 333, range: 100, direction: 1}]"

# Configuring left servos on the back board.
ros2 service call /config_servos back_board_msgs/srv/ServosConfig "servos: [{servo: 1, center: 333, range: 100, direction: -1},{servo: 2, center: 333, range: 100, direction: -1},{servo: 3, center: 333, range: 100, direction: -1}]"

# Drive two servos forward at 40% of maximum speed on the back board.
ros2 topic pub -1 /servos_proportional back_board_msgs/msg/ServoArray "{servos:[{servo: 1, value: 0.40}, {servo: 2, value: 0.40}]}"
```

> More documentation at the original page of the project (for ROS1). [Click here](https://github.com/mentor-dyun/ros-i2cpwmboard/tree/master/doc) or [here](https://gitlab.com/fmrico/ros-i2cpwmboard/-/tree/master/doc).

# Installation

You need to have ROS2 installed (of course) and theses packages provided by the default desktop installation below : 

* **rclcpp, std_msgs, std_srvs, geometry_msgs**
* **rosidl_default_generators, rosidl_default_runtime**
* Have ```python3-colcon-common-extensions``` installed
* Have ```libi2c-dev``` and ```i2c-tools``` installed
* Have the [xmlrpcpp](https://github.com/bpwilcox/xmlrpcpp) package

