> The User sub-directory

## Overview

This subdirectory is designed for each user to enter their own settings and initial values for servos and other devices. 

A brief description of the sub-directory & files:
* `config/servos_parameters.yaml`: Contains the data that is specific to every users, which will be used in the States package.
* `config/servos_params_with_single_board.yaml`: Contains the data that is specific to every users, which will be used in the States package when using a single servo (the only difference is that `use_single_board` is false).
