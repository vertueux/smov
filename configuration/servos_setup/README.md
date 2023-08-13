# The Servos Setup package

> **Note**: It requires `data/servos_parameters.yaml` to be set.

## Installation

You need to have ROS2 installed and theses packages provided by the default desktop installation below: 

* **rclcpp**.
* Have the `front_board_msgs` as well as the `back_board_msgs` package (in the `communications/` directory).

## To run it
To launch the project and the executable after compilation, you'll need to enter this command on a terminal (Note that for the moment, there is only one executable for servo control and configuration):
```bash
ros2 run servos_setup node --ros-args --params-file data/servos_parameters.yaml
```
Next, you will find a list of commands whose selection is made by entering hexadecimal numbers to choose a specific command.
