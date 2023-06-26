# The States package

> **Note** : You must have calibrated your servos before launching this package.

The States package enables the robot to be launched in real time, with all the necessary configurations, and the follow-up instructions required for each different robot, depending on its design. 

As you can see, for example, my personal configurations have been noted in the `user/example` directory, but my configurations won't necessarily work in the same way as yours, which is why it would be necessary to configure and calibrate your servos before launching this package.

## How it works

First of all, the program will set all servos to their center value. If all servos have been properly centered, they should have a position similar or exactly the same to the image below: 

## To run it 
In order to run the package with your parameters in your `user/YOUR_NAME` directory, you just need to type : 
```bash
ros2 run states basic --ros-args --params-file ~/user/YOUR_NAME/parameters.yaml
```
In example, me with my configurations I'll have to type : 
```bash
ros2 run states basic --ros-args --params-file ~/user/example/parameters.yaml
```