#!/bin/sh
# Runs the interactive container shell with the compiled applications

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

IMAGE_NAME=citrified_runtime_image
CONTAINER_NAME=citrified_runtime_container
PORT_STATE=5550
PORT_COMMAND=5551

docker run -it --rm -p"$PORT_STATE":"$PORT_STATE" -p"$PORT_COMMAND":"$PORT_COMMAND" --name $CONTAINER_NAME $IMAGE_NAME