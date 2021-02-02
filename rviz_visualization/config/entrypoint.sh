#!/bin/bash
set -e

# setup environment
source $HOME/.bashrc

# start in home directory
cd ~/ros_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin_make
cd

exec bash -i -c $@
