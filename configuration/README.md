> The Configuration sub-directory

## Overview

The configuration sub-directory contains all packages and source to the calibration & testing the servos.

This is achieved through two packages: 
* `The Calibration package` : Depends on the Board package. It facilitates independent calibration of each servo.
* `The Servos Setup package` : Depends on the Board package. It automates the setup of each servos on the `/config_servos` topic. 