#!/bin/sh

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/ROS/docker" ]; then
  echo "Run this script from within the directory CITRIFIED/ROS/docker !"
  echo "You are currently in $path"
  exit 1
fi

(
  mkdir -p ../zmq_bridge/include \
  && cp ../../control/include/franka_lwi/franka_lwi_communication_protocol.h ../zmq_bridge/include
)