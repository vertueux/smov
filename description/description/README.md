# The Description package

> The Universal Robot Description Format (URDF) is a language for describing a robot in terms of its links and joints. It is used by rviz to render robots, robot_state_publisher to convert joint states to TF2 transforms, and by Gazebo to simulate a robot. Xacro is a scripting environment for generating URDF files, which allows one to pass parameters from launch files to the URDF, as well as create more modular functions and snippets that can be incorporated into a single larger URDF.


This folder contains the `.xacro` files for some specific simulations. The meshes are included in the media distribution. This design choice enables me to load these entities from a world file.

For the time being, the description files are only supported for two models: the **Spot** and the **Spot Micro**.

## Links and References
* Spot micro URDF model : 
  * [Click here](https://gitlab.com/public-open-source/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/urdf)
* Spot URDF model : 
  * [Click here](https://github.com/clearpathrobotics/spot_ros/tree/master/spot_description)
