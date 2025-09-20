# ABV_Controller

The ABV_Controller module runs on board the air-bearing vehicles and controls the vehicle via PID feedback control

## Dependencies

The list of dependencies are in deps.sh. This file will be used by robot_idl to install the required dependencies.

## Install

This module depends on custom ROS2 msgs defined in the robot_idl repo. Clone robot_idl into a workspace and run the setup script for the vehicle as shown below.

```bash
$ mkdir -p ~/robot_ws/src
$ git clone https://github.com/samlovelace/robot_idl.git
$ cd robot_idl/scripts
$ chmod +x setup.sh
$ sudo ./setup vehicle
```

## Run

To run the abv_controller, from the root of the workspace

```bash
source install/setup.bash
ros2 run abv_controller abv_controller
```
