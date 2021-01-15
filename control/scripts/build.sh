#!/bin/sh
# Builds the runtime image (compiled applications)

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

IMAGE_NAME=citrified_runtime_image

docker build --target runtime -f ../Dockerfile --tag $IMAGE_NAME ..