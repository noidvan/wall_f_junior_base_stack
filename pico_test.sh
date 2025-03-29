#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd $SCRIPT_DIR
source /opt/ros/jazzy/setup.bash
source ./install/setup.bash
ros2 launch wall_f_junior_base_station test_system.xml &
sleep 1
ros2 run rviz2 rviz2 -d src/wall_f_junior_base_station/rviz/base_station.rviz
