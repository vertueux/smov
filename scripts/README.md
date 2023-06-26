> The Scripts sub-directory

## Overview

The scripts subdirectory contains all the `.sh` files used to run various commands to operate the SMOV robot.

> **Note** : Make sure that when you run the scripts, you are in their respective directories, otherwise it won't work.

A brief presentation of the subdirectories :
* `meshes/`: Contains scripts for copying and moving robot model `.stl` files.
* `servos/`: Contains scripts for configuring servos with predefined values (relative to each one, so you'll have to modify them for yourself) on the `/config_servos` service.
* `setup/`: Contains scripts for installing the necessary softwares (like ROS2). 
* `simulation/`: Contains the scripts needed to generate a model compatible with Gazebo simulation software.
* `urdf/`: Contains the scripts needed to generate the Spot and Spot Micro 3D models.
