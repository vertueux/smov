# The Servos Setup package

> **Note**: It requires `config/smov_dual_board.yaml.example` to be set (or `config/smov_single_board.yaml.example`).

## Installation

You need to have ROS2 installed and theses packages provided by the default desktop installation below: 

* **rclcpp**.
* Have the `i2c_pwm_board_msgs` package.

## Run it
To launch the project and the executable after compilation, you'll need to enter this command on a terminal:
```bash
ros2 run smov_setup_servos node --ros-args --params-file config/smov_dual_board.yaml.example
```
Or 
```bash
ros2 run smov_setup_servos node --ros-args --params-file config/smov_single_board.yaml.example
```
