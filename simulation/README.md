# The Simulation package

The simulation subdirectory contains the package needed to simulate the SMOV robot in a predefined environment using Gazebo software. Currently, simulation is carried out using two models: Spot and Spot Micro. You can access both launch files in the `launch/` directory.

## To run it

To run the simulation with the spot model, type the following command : 
```
ros2 launch simulation spot_simulation.launch.py
```
And to start the Spot Micro simulation, copy this command :
```
ros2 launch simulation spot_micro_simulation.launch.py
```
