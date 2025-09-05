#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/optimus/abv_ws/install/setup.bash

exec ./install/abv_controller/lib/abv_controller/abv_controller "$@"

