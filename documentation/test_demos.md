# Test demos

To make sure you've installed and configured everything correctly, you can see how to run everything normally, and try out a basic demo of a State package (the `forward_motion` package).

> **Note**: If you want to try out other demos, you can clone them from [this repository](https://github.com/vertueux/smov_demos).

## On the first terminal

Navigate to the project directory, source the installation files, then launch one of the launch files.

```bash
source /opt/ros/humble/setup.bash
cd ~/smov
source install/setup.bash
```

* If you are using only one microcontroller:

```bash
ros2 launch smov_bringup smov_single_board.launch.py
```

* If you are using two microcontrollers:

```bash
ros2 launch smov_bringup smov_dual_board.launch.py
```

You should see that your servos are at the initial values you defined earlier in your configuration file.

## On the second terminal

Navigate to the project directory, source the installation files, then run the `forward_motion` package.

```bash
source /opt/ros/humble/setup.bash
cd ~/smov
source install/setup.bash
```

You can therefore choose try a basic simulation of motion.

```bash
ros2 run forward_motion state
```

**Next step**: [Create your own State package](create_your_own_state_package.md)
