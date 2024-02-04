# The src code folder

This folder contains all the source code for the smov robot. Below is a breakdown of the folder structure

```bash
├── examples                                    -- Contains the example states for use within the smov framework
│   ├── smov_awakening                          -- A awakening state 
│   ├── smov_breath                             -- A breath state  
│   └── smov_set_legs_distance_to               -- Example code for specifying leg distances
├── hardware                                    -- Contains all the hardware driver code
│   ├── monitor                                 -- Support for a standard 16x2 LCD Module
├── launch                                      -- Contains the main launch files for smov
│   └── smov_launch                             -- The main Launch file
├── libraries                                   -- Contains the shared/static libraries used by smov
│   ├── smov_mathematics                        -- All math utility classes / functions
│   ├── smov_sequencer                          -- Sequencer library for handling state transitions
│   ├── smov_trigonometry                       -- Trig code used to modify leg positions
│   ├── smov_xmlrp                              -- XMLRP wrapper 
│   └── xmlrpcpp                                -- Raw XMLRP library
├── management                                  -- Contains all the core smov code for controlling the bot
│   ├── board                                   -- Primary node for use with direct interaction with servo controller
│   └── states                                  -- Primary node for state management within the smov bot
├── messages                                    -- Contains all the message definitions for each node/service
│   ├── board_msgs                              -- Board communication messages
│   ├── monitor_msgs                            -- Monitor communication messages
│   └── states_msgs                             -- State communication Messages
└── utilities                                   -- Contains all the utility code for configuring your bot    
    ├── calibration                             -- Calibrate the servo configuration
    └── servos_setup                            -- Setup the overall servo file

```