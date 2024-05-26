# Build the project

If you have installed ROS2 & colcon (being one of the tools to be installed), and cloned the project, simply navigate to the project directory and type the command `colcon build`:

```bash
source /opt/ros/humble/setup.bash
cd ~/smov
colcon build
```

The entire project is likely to take some time to complete. On older models of Raspberry Pi such as the 3 B+, you can use commands such as `--cmake-target [name]` or similar, to avoid freezing your device.
You can take a look at the official documentation [here](https://colcon.readthedocs.io/en/released/user/installation.html).

**Next step**: [Calibrate servos](calibrate_servos.md)
