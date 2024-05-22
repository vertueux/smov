# The Config sub-directory
This subdirectory is where the main config files are stored. It is designed for each user to enter their own settings and initial values for servos and other devices.

A brief description of the sub-directory & files:

* `config/states_dual_board.yaml.example`: Contains the data that is specific to every users, which will be used in the States package.
* `config/states_single_board.yaml.example`: Contains the data that is specific to every users, which will be used in the States package when using a single servo (the differences being that `use_single_board` is false & different ports).
