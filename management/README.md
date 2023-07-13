> The Management sub-directory

## Overview

The management sub-directory contains all packages and source code related to the base control of the SMOV robot.

This is achieved through three packages: 
* `The Board package` : Provides access to the control of both microcontrollers, and therefore directly to the servos.
* `The States package` : Depends on the Board package. It provides an initial overview of complete robot control in real time.
* `The Configuration package` : depends on the Board package. It facilitates independent configuration of each servo.

Note that for the States package, and therefore to have a preview of the robot, it is first necessary to configure (perform centering, etc...) and then annotate them. 
