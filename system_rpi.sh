#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
ros2 launch wall_f_junior_rpi system_rpi.xml
