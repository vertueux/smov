> The Mobility sub-directory

## Overview

The mobility sub-directory contains all packages and source code related to control of the SMOV robot in real time, with physical interactions.

This is achieved through two packages: 
* `The Board package` : Provides access to the control of both microcontrollers, and therefore directly to the servos.
* `The States package` : Depends on the Board package. It provides an initial overview of complete robot control in real time.

Note that for the States package, and therefore to have a preview of the robot, it is first necessary to configure (perform centering, etc...) and then annotate them. 
