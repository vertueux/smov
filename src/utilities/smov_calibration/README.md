# The Calibration package

The Calibration package lets you control servos via a command-line interface, with various choices available by entering hexadecimal numbers. It facilitates calibration of servo motors on the robot.

## Installation

You need to have ROS2 installed and theses packages provided by the default desktop installation below:

* **rclcpp**.
* Have the `i2c_pwm_board_msgs` package.

## Run it

To launch the project and the executable after compilation, you'll need to enter this command on a terminal (Note that for the moment, there is only one executable for servo control and configuration):

```bash
ros2 run smov_calibration node
```

Next, you will find a list of commands whose selection is made by entering hexadecimal numbers to choose a specific command.
