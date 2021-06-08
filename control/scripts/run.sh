#!/bin/sh
# Runs the interactive container shell with the compiled applications

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

IMAGE_NAME=citrified/control/runtime
PORT_OPTITRACK=5511
PORT_GPR=7777
PORT_JOY=8888
PORT_BRIDGE=9999

docker network inspect citrinet >/dev/null 2>&1 || \
  docker network create --subnet=172.20.0.0/16 --gateway=172.20.0.1 citrinet

docker run -it --rm \
  --network=citrinet \
  -p1601:1601 -p1602:1602 \
  -p1701:1701 -p1702:1702 \
  -p"$PORT_OPTITRACK":"$PORT_OPTITRACK" \
  -p"$PORT_GPR":"$PORT_GPR" \
  -p"$PORT_JOY":"$PORT_JOY" \
  -p"$PORT_BRIDGE":"$PORT_BRIDGE" \
  "${IMAGE_NAME}"