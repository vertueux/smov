> The Mobility sub-directory

## Overview

The mobility sub-directory contains all packages and source code related to the movement of the SMOV robot in real time, with physical interactions.

This is achieved through two packages, both independent of a central package: 
* `The Board package` : Provides access to the control of both microcontrollers, and therefore directly to the servos.
* `The Configuration package` : depends on the Board package. It facilitates independent configuration of each servo.
* `The States package` : Depends on the Board package. It provides an initial overview of complete robot control in real time.

Note that for the States package, and therefore to have a preview of the robot, it is first necessary to configure (perform centering, etc...) and then annotate them. 
