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
PORT_STATE=5550
PORT_COMMAND=5551

if [ "$REBUILD" -eq 1 ]; then
  docker build --no-cache --target debug -f ../Dockerfile --tag $IMAGE_NAME .
else
  docker build --target debug -f ../Dockerfile --tag $IMAGE_NAME .
fi

docker container stop "$CONTAINER_NAME" >/dev/null 2>&1
docker rm --force "$CONTAINER_NAME" >/dev/null 2>&1

docker run -d --cap-add sys_ptrace \
  -p127.0.0.1:"$PORT_SSH":22 -p"$PORT_STATE":"$PORT_STATE" -p"$PORT_COMMAND":"$PORT_COMMAND" \
  --name $CONTAINER_NAME $IMAGE_NAME
