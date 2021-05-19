#!/bin/sh
# Builds and runs the remote development container
# If this fails, try running the following command:
# sudo ssh-keygen -f "$HOME/.ssh/known_hosts" -R 127.0.0.1:"$PORT"
REBUILD=0
while getopts 'r' opt; do
  case $opt in
  r) REBUILD=1 ;;
  *)
    echo 'Error in command line parsing' >&2
    exit 1
    ;;
  esac
done
shift "$((OPTIND - 1))"

path=$(echo "$PWD" | rev | cut -d'/' -f-3 | rev)
if [ "$path" != "CITRIFIED/control/scripts" ]; then
  echo "Run this script from within the directory CITRIFIED/control/scripts !"
  echo "You are currently in $path"
  exit 1
fi

IMAGE_NAME=citrified/control/remote-dev
CONTAINER_NAME=citrified-control-remote-dev
PORT_SSH=2222
PORT_OPTITRACK=5511
PORT_GPR=7777
PORT_JOY=8888

if [ "$REBUILD" -eq 1 ]; then
  DOCKER_BUILDKIT=1 docker build --no-cache --target project-dependencies -f ../Dockerfile --tag $IMAGE_NAME ..
else
  DOCKER_BUILDKIT=1 docker build --target project-dependencies -f ../Dockerfile --tag $IMAGE_NAME ..
fi

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

docker network inspect citrinet >/dev/null 2>&1 || \
  docker network create --subnet=172.20.0.0/16 --gateway=172.20.0.1 citrinet

docker run -d --cap-add sys_ptrace \
  --network=citrinet \
  -p127.0.0.1:"$PORT_SSH":22 \
  -p1601:1601 -p1602:1602 \
  -p1701:1701 -p1702:1702 \
  -p"$PORT_OPTITRACK":"$PORT_OPTITRACK" \
  -p"$PORT_GPR":"$PORT_GPR" \
  -p"$PORT_JOY":"$PORT_JOY" \
  --name $CONTAINER_NAME $IMAGE_NAME
