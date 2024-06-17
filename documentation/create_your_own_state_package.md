# Create your own State package

Source the installation files from the project directory.

```bash
source /opt/ros/humble/setup.bash
source smov/install/setup.bash
```

> **Note**: I personally recommend to add these in `~/.bashrc` to make it easier for you to develop State packages in the future.

Create a directory where you will store your State packages.

```bash
mkdir state_workspace/
cd state_workspace/
```

Clone the [State template](https://github.com/vertueux/smov_state) that you will use to create your State package.

```bash
git clone https://github.com/vertueux/smov_state
```

You can now modify the package as you wish, by renaming it and adding new features.

## Run it properly

Here is the general method to run your State package properly with the SMOV States package.

### On the first terminal

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

### On the second terminal

Source the installation files from the project directory.

```bash
source /opt/ros/humble/setup.bash
source smov/install/setup.bash
```

Navigate to your workspace directory, source the installation files (after building it with the command `colcon build`), then run your package package.

```bash
cd state_workspace/
source install/setup.bash
ros2 run [state_name] state 
```

**Next step**: Have fun with your robot.
