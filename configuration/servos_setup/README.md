# The Servos Setup package

> **Note**: It requires `data/servos_parameters.yaml` to be set (or `data/servos_params_with_single_board.yaml`).

## Installation

You need to have ROS2 installed and theses packages provided by the default desktop installation below: 

* **rclcpp**.
* Have the `board_msgs`.

## To run it
To launch the project and the executable after compilation, you'll need to enter this command on a terminal (Note that for the moment, there is only one executable for servo control and configuration):
```bash
ros2 run smov_servos_setup node --ros-args --params-file data/servos_parameters.yaml
```
