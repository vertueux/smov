# Test demos

To make sure you've installed and configured everything correctly, you can see how to run everything normally, and try out a basic demo of a State package (the `set_legs_distance_to` package).

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
ros2 launch launch/smov_single_board.launch.py
```

* If you are using two microcontrollers:

```bash
ros2 launch launch/smov_dual_board.launch.py
```

You should see that your servos are at the initial values you defined earlier in your configuration file.

## On the second terminal

Navigate to the project directory, source the installation files, then run the `set_legs_distance_to` package.

```bash
source /opt/ros/humble/setup.bash
cd ~/smov
source install/setup.bash
```

You can therefore choose the distance you wish to put between your two legs.

```bash
ros2 run set_legs_distance_to state [distance]
```

For example,

```bash
ros2 run set_legs_distance_to state 14 # in centimeters.
```

**Next step**: [Create your own State package](create_your_own_state_package.md)
