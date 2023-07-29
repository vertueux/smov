# The Configuration package

> **Note** : Changes applied to all servos will also be applied to the other microcontroller when you switch microcontrollers.

The Configuration package lets you control servos via a command-line interface, with various choices available by entering hexadecimal numbers. It facilitates calibration of servo motors on the robot. 

Here are the different choices presented to you by default: 
```bash
Enter one of the following options:
-----------------------------------
0: Quit.
1: Switch board (1 or 2).
2: Reset all servos to center.
3: Set all servos to maximum value.
4: Set all servos to minimum value.
5: Reset all servos to 0.
6: Increase a servo by 2.
7: Decrease a servo by 2.
8: Increase a servo by 10.
9: Decrease a servo by 10.
A: Set new center value.
B: Set new minimum value.
C: Set new maximum value.
```
The package has been designed to be easily expandable, optimized and modular. It will therefore be easier for you to modify and adapt the source code to your needs. 

## Installation

You need to have ROS2 installed and theses packages provided by the default desktop installation below : 

* **rclcpp**.
* Have the `front_board_msgs` as well as the `back_board_msgs` package (in the `communications/` directory).

## To run it
To launch the project and the executable after compilation, you'll need to enter this command on a terminal (Note that for the moment, there is only one executable for servo control and configuration) :
```bash
ros2 run calibration node
```
Next, you will find a list of commands whose selection is made by entering hexadecimal numbers to choose a specific command.
