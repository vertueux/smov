
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

**Next step**: Have fun with your robot.
