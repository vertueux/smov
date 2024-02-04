# The States package

> **Note**: You must have calibrated your servos before launching this package.

The States package enables the robot to be launched in real time, with all the necessary configurations, and the
follow-up instructions required for each different robot, depending on its design.

## How it works

First of all, the program will set all servos to their predefined value.

## Run it

In order to run the package with your parameters, you just need to paste:

```bash
ros2 run smov_states manager --ros-args --params-file data/servos_parameters.yaml
```

or

```bash
ros2 run smov_states manager --ros-args --params-file data/servos_params_with_single_board.yaml
```
